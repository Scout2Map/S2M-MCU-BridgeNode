#!/usr/bin/env python3
#
# File   : gpio_mapping_db.py
# Purpose: SQLite backed store for event-to-GPIO mappings used by
#          gpio_event_node. Mirrors scout2map_event's ThresholdDB (same repo
#          family, same "operator edits it from the web settings panel,
#          value must survive a restart" requirement) so the two look and
#          behave the same way to anyone who already knows one of them.
# Author : jihoonkimtech

import os
import sqlite3

# Two supported polarities. The pin itself sits at the opposite level while
# idle, so a mapping never leaves a relay or LED floating in an undefined
# state between events.
#   trigger_high: idle LOW,  event raised -> HIGH
#   trigger_low : idle HIGH, event raised -> LOW
VALID_MODES = ("trigger_high", "trigger_low")


class GpioMappingDB:
    """Persist event_type -> GPIO pin mappings across restarts.

    No limit is placed on how many mappings a single event_type may have
    (an operator may want one event to drive several outputs), and no
    limit is placed on how many mappings share one physical pin -- the
    node applies whichever transition arrives most recently, see
    gpio_event_node.py's module docstring for that tradeoff.
    """

    def __init__(self, db_path=None):
        if db_path is None:
            db_dir = os.path.expanduser("~/.scout2map")
            os.makedirs(db_dir, exist_ok=True)
            db_path = os.path.join(db_dir, "gpio_events.db")
        else:
            parent = os.path.dirname(os.path.abspath(db_path))
            if parent:
                os.makedirs(parent, exist_ok=True)

        self.db_path = db_path
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.create_table()

    def create_table(self):
        self.conn.execute(
            """
            CREATE TABLE IF NOT EXISTS gpio_mappings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                event_type TEXT NOT NULL,
                pin INTEGER NOT NULL,
                mode TEXT NOT NULL,
                label TEXT NOT NULL DEFAULT ''
            )
            """
        )
        self.conn.commit()

    def add(self, event_type, pin, mode, label=""):
        if mode not in VALID_MODES:
            raise ValueError(f"unknown mode: {mode}")
        pin = int(pin)
        if pin < 0:
            raise ValueError(f"invalid pin: {pin}")

        cursor = self.conn.execute(
            """
            INSERT INTO gpio_mappings(event_type, pin, mode, label)
            VALUES (?, ?, ?, ?)
            """,
            (str(event_type), pin, mode, str(label or "")),
        )
        self.conn.commit()
        return cursor.lastrowid

    def remove(self, mapping_id):
        """Returns the removed row's pin, or None if id did not exist. The
        caller needs the pin back so it can decide whether any other
        mapping still claims it before releasing the GPIO line."""
        cursor = self.conn.execute(
            "SELECT pin FROM gpio_mappings WHERE id = ?",
            (int(mapping_id),),
        )
        row = cursor.fetchone()
        if row is None:
            return None

        self.conn.execute(
            "DELETE FROM gpio_mappings WHERE id = ?",
            (int(mapping_id),),
        )
        self.conn.commit()
        return int(row[0])

    def all(self):  # noqa: A003
        cursor = self.conn.execute(
            "SELECT id, event_type, pin, mode, label FROM gpio_mappings "
            "ORDER BY event_type, pin"
        )
        return [
            {
                "id": row[0],
                "event_type": row[1],
                "pin": row[2],
                "mode": row[3],
                "label": row[4],
            }
            for row in cursor.fetchall()
        ]

    def for_event_type(self, event_type):
        cursor = self.conn.execute(
            "SELECT id, pin, mode, label FROM gpio_mappings "
            "WHERE event_type = ?",
            (str(event_type),),
        )
        return [
            {"id": row[0], "pin": row[1], "mode": row[2], "label": row[3]}
            for row in cursor.fetchall()
        ]

    def pin_still_claimed(self, pin, exclude_id=None):
        """True if some mapping other than exclude_id still uses this pin."""
        if exclude_id is None:
            cursor = self.conn.execute(
                "SELECT 1 FROM gpio_mappings WHERE pin = ? LIMIT 1",
                (int(pin),),
            )
        else:
            cursor = self.conn.execute(
                "SELECT 1 FROM gpio_mappings WHERE pin = ? AND id != ? LIMIT 1",
                (int(pin), int(exclude_id)),
            )
        return cursor.fetchone() is not None

    def close(self):
        self.conn.close()
