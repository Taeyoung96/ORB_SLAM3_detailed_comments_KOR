## ORB SLAM3 code를 좀 더 수월하게 볼 수 있는 Cheat Sheet  

### Variable Naming  

- 변수가 클래스 멤버 변수일 경우 앞에 `m`을 붙입니다.  
- 변수가 포인터 변수일 경우 앞에 `p`를 붙입니다.  
- 변수의 자료형을 이용하여 이름을 짓는 경우가 많습니다.  
  예를 들어 `int`인 경우 `n`을 활용, `vector`인 경우 `v`를 활용, `bool`인 경우 `b`를 활용, `list`인 경우 `l`을 활용  
  
- `KF`는 `KeyFrame`의 줄임말입니다.  
- `LM`은 `Local Mapping`의 줄임말입니다.  
- `BA`는 `Bundle Adjustment`의 줄임말입니다.  
- `MP`는 `Map Point`의 줄임말입니다.  

- Rotation Matrix, Translation Vector의 경우 Naming을 할 때, 뒤 좌표계에서 앞 좌표계의 변환으로 Naming합니다.  
  예를 들어 `Tcw`의 경우, `w`는 world를 의미하고 `c`는 camera를 의미합니다.  
  따라서 월드 좌표계에서 카메라 좌표계로 Translation Vector를 의미합니다.  
  또한 `Rwg`의 경우, `w`는 world를 의미하고 `g`는 gyro를 의미합니다.  
  따라서 Gyro 좌표계에서 월드 좌표계로 Rotation Matrix를 의미합니다.  
  

