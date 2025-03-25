# Simulator
Related to virtual environment testing

## 변경사항

1. path 생성해주는 코드 변경해서 좀 더 촘촘하게 패쓰를 만들어줌.

2. 교차로 path 수정 

3. plugin package에 있는 c++버전 전부 수정

4. mpc_tracking_ws에 mpc 코드들 추가. 

### 기본

 rosrun mpc_tracking mpc_tracking_node.py

### 주차 신호 받으면 주차 이벤트까지 처리하는 것

rosrun mpc_tracking mpc_tracking_node.py