# DLSC-GC 플래너 알고리즘 흐름 가이드

> **기반 논문**: Park et al. (2023/2025) - Deadlock-Free Linear Safe Corridor with Guaranteed Convergence
>
> **목적**: 본 문서는 dlsc_gc_planner의 이론적 개념을 실제 구현에 매핑하여, 코드의 어느 부분이 논문의 어떤 알고리즘을 구현하는지 빠르게 파악할 수 있도록 합니다.

---

## 목차

1. [알고리즘 전체 흐름](#1-알고리즘-전체-흐름)
2. [단계별 상세 설명](#2-단계별-상세-설명)
   - [2.1 인식 및 상태 추정](#21-인식-및-상태-추정-perception--state-estimation)
   - [2.2 장애물 궤적 예측 및 팽창](#22-장애물-궤적-예측-및-팽창-trajectory-prediction--inflation)
   - [2.3 그리드 기반 다중 에이전트 경로 계획](#23-그리드-기반-다중-에이전트-경로-계획-mapf)
   - [2.4 중간 목표점 최적화](#24-중간-목표점-최적화-subgoal-optimization)
   - [2.5 충돌 제약 조건 생성](#25-충돌-제약-조건-생성-constraint-construction)
   - [2.6 궤적 최적화](#26-궤적-최적화-trajectory-optimization)
3. [주요 데이터 구조](#3-주요-데이터-구조)
4. [4족 보행 로봇 적용 가이드](#4-4족-보행-로봇-적용-가이드)

---

## 1. 알고리즘 전체 흐름

DLSC-GC 알고리즘은 다음과 같은 순서로 동작합니다:

```
[1] 인식 & 상태 추정
    ↓
[2] 장애물 궤적 예측 & 팽창
    ↓
[3] 그리드 기반 MAPF (교착 상태 해결)
    ↓
[4] 중간 목표점 최적화
    ↓
[5] 충돌 제약 조건 생성 (SFC, LSC, RSFC)
    ↓
[6] 궤적 최적화 (QP)
```

### 핵심 아이디어

- **교착 상태 방지**: PIBT 기반 그리드 플래너가 이산적인 경유점을 생성하여 에이전트들이 서로 막히지 않도록 조정
- **연속 궤적 최적화**: 이산 경유점을 가이드로 삼아, 동적 제약 조건을 만족하는 부드러운 궤적 생성
- **안전 복도 (Safe Corridor)**:
  - **SFC**: 정적 장애물 회피
  - **LSC**: 다른 에이전트와의 분리
  - **RSFC**: 동적 장애물의 불확실성을 고려한 회피

---

## 2. 단계별 상세 설명

## 2.1 인식 및 상태 추정 (Perception & State Estimation)

### 목적
에이전트 자신의 상태(위치, 속도)와 주변 장애물의 상태를 획득하고, 노이즈가 있는 측정값을 필터링합니다.

### 구현 위치

#### 📂 **파일**: `src/cmd_publisher.cpp`
#### 🔧 **함수**: `CmdPublisher::listenTF()`

**역할**:
1. **에이전트 자신의 위치 획득**
   - TF 변환을 통해 `/world` → `/cf{agent_id}` 변환 수신
   - 변수: `observed_agent_position`
   - 외부 위치 정보가 없으면 예측된 위치 사용

2. **동적 장애물 위치 획득 및 속도 추정**
   - TF 변환을 통해 각 장애물의 위치 수신
   - **Linear Kalman Filter (LKF)** 적용하여 속도 추정
   - 위치만 측정 가능한 센서에서 속도를 유도

**코드 위치**: `src/cmd_publisher.cpp:111-153`

```cpp
// 에이전트 상태 획득
tf_listener.lookupTransform(world_frame_id, agent_frame_id, ...);
observed_agent_position = transform.getOrigin();

// 장애물 상태 획득 및 LKF 적용
tf_listener.lookupTransform(world_frame_id, obstacle_frame_id, ...);
nav_msgs::Odometry obs_odom = linear_kalman_filters[oi].pose_cb(obs_pose);
```

---

#### 📂 **파일**: `src/linear_kalman_filter.cpp`
#### 🔧 **함수**: `LinearKalmanFilter::predict()`, `update()`

**역할**: 위치 측정값으로부터 속도를 추정

**상태 벡터**: `x = [p_x, p_y, p_z, v_x, v_y, v_z]^T` (6차원)

**칼만 필터 방정식**:
```
예측 단계:
  x_predict = F * x_old
  P_predict = F * P_old * F^T + Q

업데이트 단계:
  K = P_predict * H^T * (H * P_predict * H^T + R)^{-1}  (칼만 게인)
  x_estimate = x_predict + K * (y - H * x_predict)
  P_estimate = P_predict - K * H * P_predict
```

**물리적 의미**:
- 등속도 모델 가정: `v_{k+1} = v_k`, `p_{k+1} = p_k + v_k * dt`
- 위치 측정만으로 속도를 간접적으로 추정

**코드 위치**: `src/linear_kalman_filter.cpp:125-154`

---

#### 📂 **파일**: `src/agent_manager.cpp`
#### 🔧 **함수**: `AgentManager::obstacleCallback()`

**역할**: 장애물 정보를 통합하여 경로 계획기에 전달

**처리 흐름**:
1. 다중 에이전트 통신 시스템에서 장애물 정보 수신
2. 실제 동적 장애물(`DYN_REAL`)의 경우 LKF로 필터링된 상태로 교체
3. `TrajPlanner::setObstacles()`를 통해 경로 계획기로 전달

**코드 위치**: `src/agent_manager.cpp:119-134`

---

### 4족 보행 로봇 적용 시 고려사항

- **상태 벡터 축소**: 3D → 2D
  - `x = [p_x, p_y, theta, v_x, v_y, omega]^T`
  - z 좌표는 지형 높이 맵에서 별도 처리

- **센서 융합**: TF 대신 EKF/UKF 사용 권장
  - IMU + 다리 오도메트리 융합
  - 더 정확한 속도 추정 가능

- **주입 위치**: `CmdPublisher::listenTF()` 함수를 수정하거나, 별도의 상태 추정 노드 생성

---

## 2.2 장애물 궤적 예측 및 팽창 (Trajectory Prediction & Inflation)

### 목적
다른 에이전트 및 동적 장애물의 미래 위치를 예측하고, 불확실성을 고려하여 충돌 모델을 팽창시킵니다.

### 2.2.1 등속도 궤적 예측

#### 📂 **파일**: `src/traj_planner.cpp`
#### 🔧 **함수**: `TrajPlanner::obstaclePredictionWithCurrVel()`

**이론적 배경**:
가장 단순한 예측 모델로, 현재 속도가 유지된다고 가정합니다.

**예측 공식**:
```
p_obs(t) = p_obs(0) + v_obs * t
```

**구현**:
- 각 장애물에 대해 등속도 Bernstein 다항식 궤적 생성
- 변수: `obs_pred_trajs[obstacle_idx]` (예측 궤적 저장)

**코드 위치**: `src/traj_planner.cpp:273-288`

```cpp
for (size_t oi = 0; oi < N_obs; oi++) {
    obs_pred_trajs[oi].planConstVelTraj(
        obstacles[oi].position,  // 현재 위치
        obstacles[oi].velocity   // 현재 속도 (LKF로 추정됨)
    );
}
```

---

### 2.2.2 반경 팽창 (RSFC 핵심)

#### 📂 **파일**: `src/traj_planner.cpp`
#### 🔧 **함수**: `TrajPlanner::obstacleSizePredictionWithConstAcc()`

**이론적 배경**:
등속도 예측은 단순하지만, 실제로는 장애물이 가속할 수 있습니다.
최악의 경우를 대비하여 충돌 반경을 시간에 따라 팽창시킵니다.

**팽창 공식** (RSFC의 핵심):
```
r(t) = r_0 + 0.5 * a_max * t^2
```

**물리적 의미**:
- `r_0`: 물리적 반경
- `0.5 * a_max * t^2`: 최대 가속도로 움직였을 때의 최대 편차
- 운동학 공식에서 유도: `s = 0.5 * a * t^2`

**구현 세부사항**:
- 시간 구간별로 Bernstein 다항식 제어점 계산
- `obs_uncertainty_horizon`까지만 팽창 (너무 먼 미래는 과도하게 보수적)
- 변수: `obs_pred_sizes[obstacle_idx][segment_idx][control_point_idx]`

**코드 위치**: `src/traj_planner.cpp:338-368`

```cpp
// 각 시간 세그먼트 m에 대해
for (int m = 0; m < M_uncertainty; m++) {
    // 2차 다항식 계수 계산
    coef(0, 0) = 0.5 * max_acc * pow(m * dt, 2);      // 상수항
    coef(0, 1) = max_acc * m * dt * dt;                // 1차 항
    coef(0, 2) = 0.5 * max_acc * pow(dt, 2);          // 2차 항

    // Bernstein 제어점으로 변환
    control_points = coef * B_inv;
    obs_pred_sizes[oi][m][i] = obstacles[oi].radius + control_points(0, i);
}
```

---

#### 📂 **파일**: `include/obstacle.hpp`
#### 🔧 **함수**: `Obstacle::isCollided()`

**역할**: 팽창된 반경을 사용한 충돌 검사

**알고리즘**:
1. 시간 t마다 장애물 예측 위치 계산: `p_obs(t) = position + velocity * t`
2. 팽창된 반경 계산: `r(t) = r_0 + 0.5 * a_max * t^2`
3. 거리 검사: `dist < r_agent + r(t)`

**코드 위치**: `include/obstacle.hpp:26-36`

---

### 4족 보행 로봇 적용 시 고려사항

- **2D 투영**: z 성분 무시, x-y 평면에서만 예측
- **장애물 타입별 `a_max` 설정**:
  - 보행자: 1-2 m/s²
  - 차량: 3-5 m/s²
  - 다른 로봇: 사양에 따라
- **원형 충돌 모델**: 3D 타원체 대신 2D 원 사용

---

## 2.3 그리드 기반 다중 에이전트 경로 계획 (MAPF)

### 목적
**교착 상태(Deadlock) 방지**: 이산 그리드에서 충돌 없는 경유점을 생성하여,
연속 궤적 최적화가 항상 해를 찾을 수 있도록 가이드 제공

### 2.3.1 DOI (Dynamic Obstacle of Interest) 탐지

#### 📂 **파일**: `src/grid_based_planner.cpp`
#### 🔧 **함수**: `GridBasedPlanner::updateDOI()`

**역할**: 에이전트의 현재 경유점에 도달할 수 있는 동적 장애물 탐지

**알고리즘**:
1. 각 동적 장애물이 현재 경유점에 도달 가능한지 검사
   - `isCollided()` 사용 (팽창된 반경 고려)
2. 도달 가능한 장애물들 중 가장 가까운 것을 DOI로 선정
3. DOI 정보를 PIBT의 우선순위 결정에 사용

**물리적 의미**:
- DOI가 있으면 → 해당 장애물을 피해야 함
- PIBT에서 DOI에 가까운 에이전트에게 높은 우선순위 부여

**코드 위치**: `src/grid_based_planner.cpp:192-247`

```cpp
// 경유점에 도달 가능한 장애물 찾기
for (int oi = 0; oi < obstacles.size(); oi++) {
    bool is_waypoint_collided = obstacles[oi].isCollided(
        mapf_agent.current_waypoint,
        agent_radius,
        param.M * param.dt,              // 시간 범위
        param.obs_uncertainty_horizon    // 불확실성 범위
    );
    if (is_waypoint_collided) {
        doi_cands.emplace_back(obstacles[oi]);
    }
}

// 가장 가까운 장애물 선택
dyn_obs_interest.closest_obs_point = 가장_가까운_장애물_위치;
```

---

#### 📂 **파일**: `src/grid_based_planner.cpp`
#### 🔧 **함수**: `GridBasedPlanner::updateGoal()`

**역할**: DOI가 있을 경우, 안전한 방향으로 목표점 업데이트

**알고리즘** (BFS 기반):
1. 에이전트 현재 위치에서 BFS 시작
2. 각 노드의 "장애물 비용" 계산: `c(node) = Σ 1/d_i^2`
   - d_i: 장애물 i까지의 거리
3. 비용이 **감소하는** 방향으로만 확장 (경사 하강)
4. 최소 비용 노드를 새로운 목표점으로 설정

**물리적 의미**:
- 동적 장애물에서 멀어지는 방향으로 목표 조정
- 원래 경유점에 최대한 가까우면서도 안전한 지점 찾기

**코드 위치**: `src/grid_based_planner.cpp:250-299`

---

### 2.3.2 PIBT (Priority Inheritance with Backtracking)

#### 📂 **파일**: `include/mapf/pibt.hpp`, `src/mapf/pibt.cpp`
#### 🔧 **함수**: `PIBT::funcPIBT()`

**이론적 배경**:
분산형, 실시간 MAPF 알고리즘으로 교착 상태를 방지합니다.

**우선순위 정책**:
```
우선순위 = (obs_d, elapsed, init_d, tie_breaker)
```
1. `obs_d`: 가장 가까운 장애물까지 거리 (높을수록 낮은 우선순위)
   → **DOI에 가까운 에이전트가 우선**
2. `elapsed`: 목표 지점에서 대기한 시간 (낮을수록 높은 우선순위)
3. `init_d`: 목표까지 초기 거리 (낮을수록 높은 우선순위)
4. `tie_breaker`: 무작위 값 (결정론적 순서 보장)

**알고리즘**:
```
for each agent in priority order:
    1. 목표 방향으로 다음 노드 선택
    2. if 노드가 다른 에이전트 aj에 의해 점유됨:
        a. [우선순위 상속] aj를 먼저 이동시킴 (재귀 호출)
        b. if aj가 움직일 수 없음:
            → [백트래킹] 다른 노드 선택
    3. if 모든 노드가 막힘:
        → 현재 위치에 대기
```

**코드 위치**: `src/mapf/pibt.cpp:117-140`

```cpp
bool PIBT::funcPIBT(Agent *ai) {
    Node *v = planOneStep(ai);  // 목표 방향 노드 선택

    while (v != nullptr) {
        Agent *aj = occupied_now[v->id];
        if (aj != nullptr && aj != ai && aj->v_next == nullptr) {
            // 우선순위 상속: aj를 먼저 움직임
            if (!funcPIBT(aj)) {
                v = planOneStep(ai);  // 백트래킹: 다른 노드 선택
                continue;
            }
        }
        return true;  // 성공
    }

    // 실패: 현재 위치에 대기
    ai->v_next = ai->v_now;
    return false;
}
```

---

#### 📂 **파일**: `src/grid_based_planner.cpp`
#### 🔧 **함수**: `GridBasedPlanner::runMAPF()`

**역할**: MAPF 솔버 실행 및 결과 추출

**처리 흐름**:
1. 연속 좌표 → 그리드 노드 변환
2. PIBT 또는 ECBS 솔버 선택
3. 솔버 실행하여 이산 경로 획득
4. 그리드 노드 → 연속 좌표 변환
5. `plan_result`에 경유점 저장

**출력**:
- `plan_result.current_waypoint`: 다음 경유점
- `plan_result.goal_point`: 최종 목표

**코드 위치**: `src/grid_based_planner.cpp:424-453`

---

### 4족 보행 로봇 적용 시 고려사항

- **2D 그리드**: z축 무시, x-y 평면에서만 MAPF 수행
- **그리드 해상도**: 로봇의 footprint와 기동성에 맞게 조정
- **연결성**: 4방향 또는 8방향 (대각선 이동 가능 여부)
- **지형 비용**: 경사, 거칠기 등을 그리드 비용에 반영 가능

---

## 2.4 중간 목표점 최적화 (Subgoal Optimization)

### 목적
MAPF에서 생성한 이산 경유점을 **연속 공간의 실제 도달 가능한 목표점**으로 변환

### 문제 정의

MAPF는 이산 그리드 상의 경유점 `next_waypoint`를 제공하지만,
연속 공간에서는 장애물 때문에 정확히 도달하지 못할 수 있습니다.

**해결책**: 선분 상에서 가장 가까우면서도 안전한 점을 찾기

---

#### 📂 **파일**: `src/goal_optimizer.cpp`
#### 🔧 **함수**: `GoalOptimizer::solve()`

**최적화 문제**:
```
minimize    t
subject to  g(t) = (current_goal - next_waypoint) * t + next_waypoint
            g(t) ∈ SFC  (정적 장애물 회피)
            g(t) satisfies LSC  (에이전트 간 분리)
            t ∈ [0, 1]
```

**해석**:
- `t = 0`: 경유점에 완전히 도달 (최선)
- `t = 1`: 진전 없음 (최악)
- `t ∈ (0,1)`: 부분적 진전

**물리적 의미**:
- 경유점 방향으로 **최대한 가까이** 가되
- **모든 충돌 제약**을 만족하는 점

**코드 위치**: `src/goal_optimizer.cpp:7-136`

```cpp
// 결정 변수: t ∈ [0, 1]
x.add(IloNumVar(env, 0, 1 + SP_EPSILON_FLOAT));

// 목적 함수: minimize t
cost += x[0];
model.add(IloMinimize(env, cost));

// 제약 조건 1: SFC (정적 장애물 회피)
for (const auto &lsc: constraints.getSFC(M-1).convertToLSCs()) {
    // n · (g(t) - p) - d >= 0
}

// 제약 조건 2: LSC (에이전트 간 분리)
for (size_t oi = 0; oi < N_obs; oi++) {
    // n · (g(t) - p_obs) - d >= 0
}

// 해 추출
goal = (current_goal - next_waypoint) * vals[0] + next_waypoint;
```

---

#### 📂 **파일**: `src/goal_optimizer.cpp`
#### 🔧 **함수**: `GoalOptimizer::populatebyrow()`

**역할**: QP 모델 구축

**제약 조건 추가**:
1. **SFC 제약**: 정적 장애물로부터 안전한 영역
2. **LSC 제약**: 다른 에이전트와의 최소 거리 유지

**코드 위치**: `src/goal_optimizer.cpp:138-198`

---

### 4족 보행 로봇 적용 시 고려사항

- **2D 문제로 축소**: SFC는 2D 다각형, LSC는 2D 반평면
- **지형 제약 추가 가능**: 경사, 장애물 높이 등
- **실시간성**: QP는 빠르게 풀림 (밀리초 단위)

---

## 2.5 충돌 제약 조건 생성 (Constraint Construction)

### 목적
궤적 최적화를 위한 안전 복도(Safe Corridor) 구축

### 제약 조건 종류

| 제약 | 용도 | 형태 |
|------|------|------|
| **SFC** | 정적 장애물 회피 | 축 정렬 박스 (AABB) |
| **LSC** | 에이전트 간 분리 | 반평면 (GJK 기반) |
| **RSFC** | 동적 장애물 회피 | 반평면 (접평면 기반) |

---

### 2.5.1 SFC (Static Flight Corridor) 생성

#### 📂 **파일**: `src/collision_constraints.cpp`
#### 🔧 **함수**: `CollisionConstraints::expandSFCIncrementally()`

**이론적 배경**:
정적 장애물이 없는 최대 크기의 축 정렬 박스를 생성합니다.

**알고리즘** (6방향 축 탐색):
```
1. 초기 박스: 에이전트 위치를 포함하는 작은 박스
2. for each direction in {-x, -y, -z, +x, +y, +z}:
       while 박스가 경계 안에 있고 && 장애물 없음:
           해당 방향으로 1 그리드 해상도만큼 확장
       확장 중단 (장애물 발견)
3. 마진 보상 적용
```

**충돌 검사**:
- Euclidean Distance Transform (EDT) 맵 사용
- 각 셀의 거리값: `dist < agent_radius`이면 충돌

**수학적 정의**:
```
SFC = {c ∈ R³ | box_min < c < box_max}

6개의 반평면 제약으로 변환:
  c_x > box_min_x,  c_x < box_max_x
  c_y > box_min_y,  c_y < box_max_y
  c_z > box_min_z,  c_z < box_max_z
```

**코드 위치**: `src/collision_constraints.cpp:1023-1093`

```cpp
// 6방향 확장 후보
std::vector<int> axis_cand = {0, 1, 2, 3, 4, 5};
// 0=-x, 1=-y, 2=-z, 3=+x, 4=+y, 5=+z

while (!axis_cand.empty()) {
    while (경계_안에_있고 && !장애물_있음) {
        // 현재 방향으로 확장
        if (axis < 3) {
            sfc_cand.box_min(axis) -= world_resolution;
        } else {
            sfc_cand.box_max(axis-3) += world_resolution;
        }
    }
    // 이 방향은 더 이상 확장 불가
    axis_cand.erase(current_axis);
}
```

---

### 2.5.2 LSC (Linear Safe Corridor) 생성 - 에이전트 간

#### 📂 **파일**: `src/traj_planner.cpp`
#### 🔧 **함수**: `TrajPlanner::generateDLSCGC()`

**이론적 배경** (DLSC-GC의 핵심):
- 일반 LSC: 현재 궤적 세그먼트만 고려
- **DLSC-GC**: 마지막 세그먼트에서 **목표 기반 분리** 추가 → 교착 상태 방지

**수학적 정의**:
```
LSC = {c ∈ R³ | (c - c_obs) · n - d > 0}

여기서:
  c: 에이전트 제어점 (결정 변수)
  c_obs: 장애물 제어점
  n: 분리 초평면 법선 벡터
  d: 안전 마진
```

**알고리즘**:
```
for each obstacle:
    if obstacle.type == AGENT:
        for m = 0 to M-2:  // 중간 세그먼트
            n = normalVectorBetweenPolys(GJK 사용)
            d = 0.5 * (collision_dist + projection)

        for m = M-1:  // 마지막 세그먼트 (GC 컴포넌트)
            목표로 향하는 선분 간 최근접점 계산
            n = separation direction
            d = based on goal-to-goal distance

    else:  // 동적 장애물
        n = normalVectorDynamicObs(접평면)
        d = r_obs(t) + r_agent
```

**코드 위치**: `src/traj_planner.cpp:700-763`

---

#### 📂 **파일**: `src/traj_planner.cpp`
#### 🔧 **함수**: `TrajPlanner::normalVectorBetweenPolys()`

**역할**: GJK 알고리즘을 사용한 분리 법선 벡터 계산

**GJK 알고리즘**:
1. **상대 제어점** 계산:
   ```
   rel_i = agent_ctrl_pt_i - obstacle_ctrl_pt_i
   ```

2. 상대 제어점들의 **볼록 껍질(Convex Hull)** 형성
   - 이는 민코프스키 차(Minkowski Difference)

3. 원점에서 볼록 껍질까지의 **최근접점** 찾기 (GJK)
   - 원점이 밖에 있으면 → 충돌 없음
   - 원점이 안에 있으면 → 잠재적 충돌

4. **법선 벡터** = 원점에서 최근접점 방향
   ```
   n = closest_point.normalized()
   ```

**기하학적 해석**:
- 두 궤적의 볼록 껍질이 분리되어 있으면 안전
- 법선 벡터는 최적 분리 방향

**코드 위치**: `src/traj_planner.cpp:1199-1235`

```cpp
// 상대 제어점 계산
for (size_t i = 0; i < n_control_points; i++) {
    control_points_rel[i] =
        initial_traj_trans[m][i] - obs_pred_traj_trans[m][i];
}

// GJK: 원점에서 볼록 껍질까지 최근접점
ClosestPoints closest_points =
    closestPointsBetweenPointAndConvexHull(
        point3d(0, 0, 0),
        control_points_rel
    );

// 법선 벡터 = 분리 방향
point3d normal_vector = closest_points.closest_point2.normalized();
```

---

### 2.5.3 RSFC (Relative Safe Flight Corridor) 생성

#### 📂 **파일**: `src/traj_planner.cpp`
#### 🔧 **함수**: `TrajPlanner::generateReciprocalRSFC()`

**이론적 배경**:
동적 장애물의 불확실성을 고려한 충돌 회피

**접평면 방법**:
```
1. 장애물과 에이전트 궤적을 선분으로 근사
2. 두 선분 간 최근접점 계산
3. 법선 벡터 = 장애물 선분에서 에이전트 선분 방향
4. 안전 마진 d = r_obs(t) + r_agent
   또는 (reciprocal) d = 0.5 * (r_obs + r_agent + dist)
```

**Reciprocal 개념**:
- 두 에이전트가 가까울 때, 회피 책임을 **절반씩 분담**
- 한쪽만 피하지 않고 양쪽이 조금씩 양보

**코드 위치**: `src/traj_planner.cpp:624-650`

```cpp
// 선분 근사
Line obs_path(obs_pred_trajs[oi][m][0], obs_pred_trajs[oi][m][n]);
Line agent_path(initial_traj[m][0], initial_traj[m][n]);

// 법선 벡터 계산
normal_vector = normalVectorBetweenLines(obs_path, agent_path, closest_dist);

// 안전 마진
if (obstacles[oi].type == AGENT && closest_dist < r_total) {
    // Reciprocal: 절반씩 분담
    d = 0.5 * (r_obs + r_agent + closest_dist);
} else {
    d = r_obs(t) + r_agent;  // 팽창된 반경 사용
}
```

---

#### 📂 **파일**: `src/traj_planner.cpp`
#### 🔧 **함수**: `TrajPlanner::normalVectorDynamicObs()`

**역할**: 동적 장애물에 대한 접평면 법선 계산

**전략**:
1. 목표 방향과 현재 방향이 일치하면
   → 간단한 반발력 (장애물에서 에이전트 방향)
2. 그렇지 않으면
   → 목표 방향으로 유도 (장애물 주변을 돌아감)

**코드 위치**: `src/traj_planner.cpp:1314-1350`

---

### 4족 보행 로봇 적용 시 고려사항

#### SFC
- **2D 직사각형**: 4방향 확장 (-x, -y, +x, +y)
- z축은 지형 높이로 별도 처리

#### LSC/RSFC
- **2D 반평면**: `n_x * (x - x_obs) + n_y * (y - y_obs) - d > 0`
- GJK를 2D 평면에서 수행
- 원형 충돌 모델 (타원체 대신)

---

## 2.6 궤적 최적화 (Trajectory Optimization)

### 목적
생성된 제약 조건(SFC, LSC, RSFC)을 만족하면서 부드러운 궤적 생성

#### 📂 **파일**: `src/traj_optimizer.cpp`
#### 🔧 **함수**: `TrajOptimizer::solve()`

**QP 문제 정의**:
```
minimize    J = ∫ ||ϕ차_미분||² dt + terminal_cost + slack_penalty

subject to
    - SFC: c[m][i] ∈ Box[m]
    - LSC: n · (c[m][i] - c_obs[m][i]) - d >= 0
    - RSFC: n · (c[m][i] - c_obs[m][i]) - d >= 0
    - 동적 제약: v_max, a_max 등
    - 연속성: 세그먼트 간 연결
```

**비용 함수**:
1. **제어 노력**: 저크(jerk) 최소화 → 부드러운 궤적
2. **목표 비용**: 마지막 지점이 목표에 가까이
3. **슬랙 페널티**: 제약 위반 시 큰 패널티

**출력**:
- `desired_traj`: M개 세그먼트, 각 n+1개 제어점
- Bernstein 다항식으로 표현된 궤적

**코드 위치**: `src/traj_optimizer.cpp:18-110`

---

### 4족 보행 로봇 적용 시 고려사항

- **2D 궤적**: x, y만 최적화, z는 지형 추종
- **방향 제약 추가**: 로봇의 방향(yaw) 최적화
- **동적 제약 조정**: 4족 보행의 속도/가속도 제한
- **지형 비용**: 경사 페널티 추가 가능

---

## 3. 주요 데이터 구조

### 3.1 궤적 표현

```cpp
// Bernstein 다항식 궤적
Trajectory<point3d> traj(M, n, dt);
// M: 세그먼트 수
// n: 다항식 차수
// dt: 세그먼트 시간

// 접근: traj[segment_idx][control_point_idx]
point3d ctrl_pt = traj[5][3];  // 5번 세그먼트의 3번 제어점
```

### 3.2 장애물

```cpp
struct Obstacle {
    point3d position;      // 현재 위치
    point3d velocity;      // LKF로 추정된 속도
    double radius;         // 물리적 반경
    double max_acc;        // 최대 가속도 (RSFC용)
    ObstacleType type;     // AGENT, DYN_REAL, 등
};
```

### 3.3 제약 조건

```cpp
// LSC (반평면 제약)
class LSC {
    point3d obs_control_point;  // 장애물 참조점
    point3d normal_vector;       // 법선 벡터
    double d;                    // 안전 마진

    // 제약: (c - obs_control_point) · normal_vector - d > 0
};

// SFC (박스 제약)
class Box {
    point3d box_min;
    point3d box_max;

    // 제약: box_min < c < box_max
};
```

---

## 4. 4족 보행 로봇 적용 가이드

### 4.1 주요 수정 사항 요약

| 단계 | 3D UAV | 2D 4족 보행 로봇 | 수정 위치 |
|------|---------|------------------|-----------|
| **상태 벡터** | `[x,y,z, vx,vy,vz]` | `[x,y,θ, vx,vy,ω]` | `linear_kalman_filter.hpp` |
| **센서** | TF | IMU + 다리 오도메트리 | `cmd_publisher.cpp::listenTF()` |
| **예측** | 3D 등속도 | 2D 등속도 | `traj_planner.cpp::obstaclePredictionWithCurrVel()` |
| **충돌 모델** | 타원체 | 원 | `obstacle.hpp::isCollided()` |
| **MAPF** | 3D 그리드 | 2D 그리드 | `grid_based_planner.cpp` |
| **SFC** | AABB (6면) | 직사각형 (4면) | `collision_constraints.cpp` |
| **LSC/RSFC** | 3D 반평면 | 2D 반평면 | `traj_planner.cpp::generateDLSCGC()` |
| **궤적 최적화** | 3D (x,y,z) | 2D (x,y) + yaw | `traj_optimizer.cpp` |

---

### 4.2 단계별 적용 가이드

#### Step 1: 상태 추정 수정
**파일**: `include/linear_kalman_filter.hpp`, `src/cmd_publisher.cpp`

```cpp
// 상태 벡터 재정의
// Before: [p_x, p_y, p_z, v_x, v_y, v_z]
// After:  [p_x, p_y, theta, v_x, v_y, omega]

// EKF/UKF 사용 권장
// - IMU: 각속도, 가속도
// - 다리 오도메트리: 속도 추정
// - 비전/LiDAR: 위치 보정
```

#### Step 2: 2D 예측 및 충돌 검사
**파일**: `src/traj_planner.cpp`, `include/obstacle.hpp`

```cpp
// obstaclePredictionWithCurrVel()에서
// z 성분 무시 또는 지형 높이로 설정
obs_pred_trajs[oi].planConstVelTraj2D(position_2d, velocity_2d);

// isCollided()에서 2D 거리 사용
double dist_2d = sqrt(dx*dx + dy*dy);
```

#### Step 3: 2D MAPF
**파일**: `src/grid_based_planner.cpp`

```cpp
// 그리드 해상도 조정
param.grid_resolution = 0.5;  // 로봇 크기에 맞게

// 4방향 또는 8방향 연결
// 대각선 이동 가능 여부에 따라
```

#### Step 4: 2D SFC
**파일**: `src/collision_constraints.cpp`

```cpp
// expandSFCIncrementally()에서
// 4방향만 확장: {-x, -y, +x, +y}
std::vector<int> axis_cand = {0, 1, 3, 4};

// z축 고정
box_min.z() = terrain_height;
box_max.z() = terrain_height + robot_height;
```

#### Step 5: 2D LSC/RSFC
**파일**: `src/traj_planner.cpp`

```cpp
// normalVectorBetweenPolys()에서
// z 성분 제거
control_points_rel[i].z() = 0;

// 2D GJK 사용
ClosestPoints closest_points_2d =
    closestPointsBetweenPointAndConvexHull2D(...);
```

#### Step 6: 2D 궤적 최적화
**파일**: `src/traj_optimizer.cpp`

```cpp
// QP 문제를 2D로 축소
// 결정 변수: [x, y, yaw] 제어점

// 방향 제약 추가 (선택)
// - 급격한 yaw 변화 방지
// - 전방 선호 방향
```

---

### 4.3 파라미터 튜닝

**파일**: `launch/simulation.launch`

```xml
<!-- 2D 설정 -->
<param name="world/dimension" value="2" />

<!-- 그리드 해상도 -->
<param name="grid/resolution" value="0.5" />  <!-- 로봇 크기의 2배 권장 -->

<!-- 동적 제약 -->
<param name="agent/max_vel" value="1.0" />    <!-- 4족 보행 속도 -->
<param name="agent/max_acc" value="2.0" />    <!-- 4족 보행 가속도 -->

<!-- 장애물 불확실성 -->
<param name="obs/uncertainty_horizon" value="1.0" />
<param name="obs/max_acc" value="2.0" />      <!-- 보행자/차량 가속도 -->

<!-- 궤적 표현 -->
<param name="traj/M" value="10" />            <!-- 세그먼트 수 -->
<param name="traj/dt" value="0.2" />          <!-- 세그먼트 시간 -->
<param name="traj/n" value="5" />             <!-- 다항식 차수 -->
```

---

### 4.4 추가 고려사항

#### 지형 처리
```cpp
// 별도의 지형 높이 맵 사용
double getTerrainHeight(double x, double y);

// 경사 비용
double getSlopeCost(double x, double y);
```

#### 방향 제약
```cpp
// yaw 최적화 추가
// - 목표 방향 선호
// - 급격한 회전 방지
```

#### 다리 간섭
```cpp
// 회전 반경 고려
// - 제자리 회전 vs 회전 반경
```

---

## 5. 전체 데이터 흐름

```
[입력]
├─ TF: /world → /robot
├─ TF: /world → /obstacle_i
└─ 목표 위치

    ↓ [CmdPublisher::listenTF()]

[LKF 필터링]
├─ observed_agent_position
└─ observed_obs_odoms[i] = {position, velocity}

    ↓ [AgentManager::obstacleCallback()]

[장애물 정보 통합]
└─ obstacles[] → TrajPlanner

    ↓ [TrajPlanner::obstaclePredictionWithCurrVel()]
        [TrajPlanner::obstacleSizePredictionWithConstAcc()]

[예측 궤적 & 팽창 반경]
├─ obs_pred_trajs[i]: 등속도 궤적
└─ obs_pred_sizes[i]: r(t) = r_0 + 0.5*a_max*t²

    ↓ [GridBasedPlanner::updateDOI()]
        [GridBasedPlanner::updateGoal()]
        [GridBasedPlanner::runMAPF()]

[PIBT 경로 계획]
├─ current_waypoint: 다음 경유점
└─ goal_point: 최종 목표

    ↓ [GoalOptimizer::solve()]

[중간 목표 최적화]
└─ current_goal_point: 실제 도달 가능한 목표

    ↓ [CollisionConstraints::expandSFCIncrementally()]
        [TrajPlanner::generateDLSCGC()]
        [TrajPlanner::generateReciprocalRSFC()]

[제약 조건 생성]
├─ SFC: 정적 장애물 회피 박스
├─ LSC: 에이전트 간 분리 반평면 (GJK)
└─ RSFC: 동적 장애물 회피 반평면 (접평면)

    ↓ [TrajOptimizer::solve()]

[궤적 최적화 (QP)]
└─ desired_traj: 최종 궤적

    ↓

[출력]
└─ 제어 명령 (position, velocity, acceleration)
```

---

## 6. 핵심 함수 빠른 참조

### 인식 & 상태 추정
| 기능 | 파일 | 함수 |
|------|------|------|
| 에이전트 위치 획득 | `cmd_publisher.cpp` | `listenTF()` |
| LKF 예측 | `linear_kalman_filter.cpp` | `predict()` |
| LKF 업데이트 | `linear_kalman_filter.cpp` | `update()` |
| 불확실성 반경 | `kalman_filter.hpp` | `getUncertaintyRadius()` |

### 예측 & 팽창
| 기능 | 파일 | 함수 |
|------|------|------|
| 등속도 예측 | `traj_planner.cpp` | `obstaclePredictionWithCurrVel()` |
| 반경 팽창 | `traj_planner.cpp` | `obstacleSizePredictionWithConstAcc()` |
| 충돌 검사 | `obstacle.hpp` | `isCollided()` |

### MAPF
| 기능 | 파일 | 함수 |
|------|------|------|
| DOI 탐지 | `grid_based_planner.cpp` | `updateDOI()` |
| 목표 업데이트 | `grid_based_planner.cpp` | `updateGoal()` |
| PIBT 실행 | `pibt.cpp` | `funcPIBT()` |
| MAPF 래퍼 | `grid_based_planner.cpp` | `runMAPF()` |

### 목표 최적화
| 기능 | 파일 | 함수 |
|------|------|------|
| 중간 목표 최적화 | `goal_optimizer.cpp` | `solve()` |
| QP 모델 구축 | `goal_optimizer.cpp` | `populatebyrow()` |

### 제약 조건
| 기능 | 파일 | 함수 |
|------|------|------|
| SFC 확장 | `collision_constraints.cpp` | `expandSFCIncrementally()` |
| DLSC-GC 생성 | `traj_planner.cpp` | `generateDLSCGC()` |
| RSFC 생성 | `traj_planner.cpp` | `generateReciprocalRSFC()` |
| GJK 법선 | `traj_planner.cpp` | `normalVectorBetweenPolys()` |
| 동적 장애물 법선 | `traj_planner.cpp` | `normalVectorDynamicObs()` |

### 궤적 최적화
| 기능 | 파일 | 함수 |
|------|------|------|
| QP 궤적 최적화 | `traj_optimizer.cpp` | `solve()` |

---

## 7. 디버깅 팁

### 7.1 각 단계별 시각화

```cpp
// ROS 파라미터로 로깅 활성화
<param name="log/enable_visualization" value="true" />

// 각 단계 결과를 RViz에서 확인
- /sfc_visualization: 정적 안전 복도
- /lsc_visualization: 에이전트 간 분리 평면
- /predicted_trajectories: 예측 궤적
- /grid_path: PIBT 경로
```

### 7.2 CPLEX 디버깅

```cpp
// QP 모델 저장
<param name="log/solver" value="true" />

// 생성 파일:
- log/QPmodel_goalOpt.lp: 목표 최적화 문제
- log/QPmodel_traj.lp: 궤적 최적화 문제
- log/conflict_*.lp: 제약 충돌 시 진단
```

### 7.3 일반적인 문제

| 증상 | 원인 | 확인 위치 |
|------|------|-----------|
| 에이전트 멈춤 | QP 해 없음 | `traj_optimizer.cpp` 로그 |
| 진동 | 제약 너무 타이트 | LSC/RSFC 마진 확인 |
| 교착 상태 | PIBT 실패 | `pibt.cpp` 우선순위 |
| 충돌 | SFC 너무 큼 | `expandSFCIncrementally()` 마진 |

---

## 8. 요약

1. **인식**: TF + LKF로 에이전트 및 장애물 상태 추정
2. **예측**: 등속도 모델 + 반경 팽창 (`r(t) = r_0 + 0.5*a*t²`)
3. **MAPF**: PIBT로 교착 없는 경유점 생성
4. **목표 최적화**: 경유점 방향으로 안전한 중간 목표 찾기
5. **제약 생성**: SFC (정적) + LSC (에이전트) + RSFC (동적)
6. **궤적 최적화**: QP로 부드러운 궤적 생성

**핵심 논문 기여**:
- DLSC-GC: 목표 기반 분리 → 교착 방지
- RSFC: 불확실성 고려 → 안전한 동적 장애물 회피

**4족 보행 적용**:
- 3D → 2D 투영
- 지형 고려
- 동적 제약 조정

---

**문서 버전**: 1.0
**작성일**: 2025
**기반 코드**: dlsc_gc_planner (Park et al. 2023/2025)
