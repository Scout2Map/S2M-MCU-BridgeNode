# resource/

`scout2map_bridge`는 ament 인덱스 마커 파일이다. **0바이트가 정상이며,
파일 이름이 패키지 이름과 정확히 일치해야 한다.** `setup.py`가 이 경로를
그대로 읽는다.

이 파일이 없으면 빌드가 다음과 같이 실패한다.

```
error: could not create '.../ament_index/resource_index/packages/scout2map_bridge':
No such file or directory
```

0바이트 파일은 압축·복사·전송 과정에서 누락되는 일이 잦다.
사라졌다면 다시 만들면 된다.

```bash
touch resource/scout2map_bridge
```

이 README는 디렉토리 자체가 비어 보이지 않게 하여 누락을 줄이려는 목적도 겸한다.
