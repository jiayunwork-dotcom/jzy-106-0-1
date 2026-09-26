# Attitude Integration Service

刚体姿态的**四元数积分常驻服务**（Go 1.22 + Gin）。只做姿态本身：
给定一段随时间变化的**机体角速度**序列，沿时间序列一步步推进姿态四元数，
返回末端姿态、末端欧拉角和全程范数漂移。不做位置/速度/惯导滤波，
没有任何页面，欧拉角只在输出时换算。

## 运动学约定

- 四元数为标量在前的 Hamilton 约定 `q = [w, x, y, z]`，单位四元数绕单位轴
  `a` 转 `θ`：`q = (cos θ/2, a sin θ/2)`。
- 角速度在**机体系**下，运动学方程为**右乘**：

  ```
  dq/dt = 1/2 · q ⊗ (0, ω_b)
  ```

  （参考系角速度才是左乘，不可弄反。）

- 推进方法二选一：
  - `rk4`（默认）：四阶经典 Runge–Kutta，每步推进后重新归一化；
  - `exp`：指数映射（零阶保持），`q ← q ⊗ exp(ω·dt/2)`，本身保范。
- 每一步后**重新归一化**，并测量归一化前的单步范数漂移 `|1 − |q*||`：
  超过 `warn_norm_drift`（默认 1e-6）记告警，超过 `reject_norm_drift`
  （默认 1e-2）直接以 `422` 拒绝，不会闷头把姿态积失真。
- 欧拉角**全服务固定一种约定**：**ZYX 内旋**（intrinsic z-y-x，
  即 yaw-pitch-roll，`R = Rz(ψ)Ry(θ)Rx(φ)`），弧度，响应里写明
  `"convention":"ZYX"`。俯仰接近 ±90° 奇异区时返回 `singular=true`
  与中文告警，按 `roll=0` 约定解出有限的 yaw，绝不输出 NaN。
- 时间输入二选一：`time_steps`（N 个角速度配 N 个正步长），或
  `timestamps`（N 个角速度配 **N+1** 个严格递增时间戳，服务自行差分）。

## 目录结构（按职责分文件）

```
cmd/attitude-server/main.go     服务入口
internal/quaternion/quaternion.go   四元数代数：乘法/归一化/共轭/指数映射
internal/integrator/integrator.go   RK4 与指数映射推进（与代数分离）
internal/euler/euler.go             四元数↔ZYX 欧拉角换算与奇异检测
internal/validation/validation.go   积分开始前的参数合法性校验
internal/store/store.go             命名角速度序列的内存存取（重启即丢）
internal/api/                       轻薄接口层：解析请求/驱动积分/组织响应
```

## 构建与启动（Docker 一条命令）

```bash
docker compose up --build
# 或：
docker build -t attitude-service . && \
  docker run --rm -p 8080:8080 attitude-service
```

对外接口：`POST http://localhost:8080/api/v1/attitude/integrate`。
端口可用环境变量 `ATTITUDE_HTTP_ADDR`（或 `PORT`）覆盖。

## API

### 1) 直接积分 `POST /api/v1/attitude/integrate`

```json
{
  "initial_quaternion": [1, 0, 0, 0],
  "angular_velocities": [[0,0,0.5],[0,0,0.5]],
  "time_steps": [0.1, 0.1],
  "method": "rk4",
  "include_trace": false
}
```

也可用 `"timestamps": [0.0, 0.1, 0.2]` 代替 `time_steps`（长度必须 N+1）。

响应：

```json
{
  "final_quaternion": [0.99875026, 0, 0, 0.04997917],
  "final_euler": {"roll":0, "pitch":0, "yaw":0.1,
                  "convention":"ZYX", "singular":false},
  "steps": 2,
  "duration": 0.2,
  "final_norm": 1.0,
  "final_norm_drift": 0.0,
  "max_step_norm_drift": 1.7e-12,
  "warnings": []
}
```

`include_trace:true` 时额外返回每个时间点（共 N+1 点）的四元数与欧拉角。

### 2) 命名序列（运行期复用，重启即丢）

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/v1/sequences?name=bench-yaw` | 保存序列（body 只需角速度+时间） |
| GET | `/api/v1/sequences` | 列出全部序列元信息 |
| GET | `/api/v1/sequences/:name` | 取序列详情 |
| DELETE | `/api/v1/sequences/:name` | 删除 |
| POST | `/api/v1/sequences/:name/integrate` | 用已存序列积分，body 只给初始姿态/配置 |

### 错误

- `400 INVALID_PARAMETER`：空序列、初始四元数非单位、时间步长非正、
  采样数与时间戳/步长数对不上、时间戳非递增、方法未知等，响应带具体原因；
- `404 SEQUENCE_NOT_FOUND`；
- `422 STEP_NORM_DRIFT_EXCEEDED`：单步范数漂移超拒绝阈值（步长太大）。

## 被自动化测试锁死的运动学事实

`go test ./...`（Docker 构建阶段也会先跑一遍）：

1. **角速度加倍 + 时间步长减半 → 末端姿态不变**（总转角不变）；
2. **整段角速度反号 → 末端姿态为原过程之逆**；另含“时间倒序+反号倒放
   回到起点”的一般多轴验证；
3. **恒定角速度绕固定轴 → 转角 = |ω|·t**；
4. 绕固定轴转**整整一圈 → 与初始朝向相同**（允许四元数整体差一个符号，
   用 SO(3) 测地距离判定）；
5. **零角速度 → 姿态冻结**；
6. 预置手算基准：纯绕竖直 z 轴匀速 0.5 rad/s × 1 s，
   **yaw 随时间线性增长（最小二乘斜率 0.5）、pitch/roll 保持近零**，
   逐点钉进回归测试。

另有并发隔离测试（多段序列同时推进，`-race` 通过）、奇异告警、
大步长 422 拒绝、非法输入 400 与命名序列全生命周期的 HTTP 端到端测试。

## 本地开发

```bash
go test ./...           # 全量测试
go test -race ./...     # 竞争检测
go run ./cmd/attitude-server
```
