package integrator

import (
	"fmt"
	"math"
	"sync"
	"testing"

	"attitude-service/internal/euler"
	"attitude-service/internal/quaternion"
)

// 工具：生成 n 步恒定角速度序列。
func constSamples(w quaternion.Vec3, dt float64, n int) []Sample {
	s := make([]Sample, n)
	for i := range s {
		s[i] = Sample{Omega: w, Dt: dt}
	}
	return s
}

func scaleSamples(s []Sample, wScale, dtScale float64) []Sample {
	out := make([]Sample, len(s))
	for i, x := range s {
		out[i] = Sample{
			Omega: quaternion.ScaleVec(x.Omega, wScale),
			Dt:    x.Dt * dtScale,
		}
	}
	return out
}

func reverseNegated(s []Sample) []Sample {
	out := make([]Sample, len(s))
	for i, x := range s {
		out[len(s)-1-i] = Sample{Omega: quaternion.ScaleVec(x.Omega, -1), Dt: x.Dt}
	}
	return out
}

// 事实 1：角速度全部放大一倍、每个时间步长减半，末端姿态必须一致
// （转过的总角度不变，且每一步 ω·dt 完全相同）。多轴、变步长构造。
func TestScaleDoubleTimeHalf_FinalAttitudeInvariant(t *testing.T) {
	base := []Sample{
		{Omega: quaternion.Vec3{X: 1.0, Y: 0.5, Z: -0.3}, Dt: 0.011},
		{Omega: quaternion.Vec3{X: 0.2, Y: 0.8, Z: 0.4}, Dt: 0.013},
		{Omega: quaternion.Vec3{X: -0.5, Y: 0.1, Z: 0.9}, Dt: 0.009},
		{Omega: quaternion.Vec3{X: 0.7, Y: -0.2, Z: 0.33}, Dt: 0.012},
		{Omega: quaternion.Vec3{X: -0.1, Y: 0.6, Z: -0.4}, Dt: 0.010},
		{Omega: quaternion.Vec3{X: 0.3, Y: 0.3, Z: 0.3}, Dt: 0.014},
		{Omega: quaternion.Vec3{X: 0.9, Y: -0.7, Z: -0.1}, Dt: 0.008},
	}
	doubled := scaleSamples(base, 2.0, 0.5)

	for _, method := range []string{MethodRK4, MethodExp} {
		cfg := Config{Method: method, WarnNormDrift: 1e-4, RejectNormDrift: 1}
		a, err := Integrate(quaternion.Identity(), base, cfg)
		if err != nil {
			t.Fatalf("method=%s base: %v", method, err)
		}
		b, err := Integrate(quaternion.Identity(), doubled, cfg)
		if err != nil {
			t.Fatalf("method=%s doubled: %v", method, err)
		}
		if d := quaternion.GeodesicDistance(a.Final, b.Final); d > 1e-12 {
			t.Errorf("method=%s: 加倍角速度+减半步长后末端姿态偏差 %.3e 超过 1e-12", method, d)
		}
	}
}

// 事实 2：整段角速度全部反号，末端姿态是原过程的逆转。
// 对固定轴（单轴旋转可交换）从恒等姿态出发，q(-ω) 必须等于 q(ω)^{-1}=q(ω)*。
func TestNegateAngularVelocity_FixedAxisIsInverse(t *testing.T) {
	w := quaternion.Vec3{X: 0.4, Y: -0.9, Z: 0.7}
	fwd := constSamples(w, 0.01, 60)
	neg := scaleSamples(fwd, -1, 1)

	for _, method := range []string{MethodRK4, MethodExp} {
		cfg := Config{Method: method}
		qf, err := Integrate(quaternion.Identity(), fwd, cfg)
		if err != nil {
			t.Fatalf("method=%s fwd: %v", method, err)
		}
		qn, err := Integrate(quaternion.Identity(), neg, cfg)
		if err != nil {
			t.Fatalf("method=%s neg: %v", method, err)
		}
		if d := quaternion.GeodesicDistance(qn.Final, qf.Final.Conj()); d > 1e-11 {
			t.Errorf("method=%s: 反号序列末端姿态与原姿态之逆偏差 %.3e", method, d)
		}
		// 确保用例非平凡：正向确实转过了不可忽略的角度。
		if ang := quaternion.RotationAngle(qf.Final); ang < 0.1 || ang > math.Pi {
			t.Errorf("method=%s: 用例转角 %.4f 不在 (0.1, π)，断言失去意义", method, ang)
		}
	}
}

// 事实 2（一般多轴情形的严格版）：沿原路径“倒放”——时间倒序且角速度反号，
// 复合旋转恰为原过程的逆；从原末端姿态出发倒放必须回到初始姿态。
func TestReverseRetrace_ReturnsToStart(t *testing.T) {
	base := []Sample{
		{Omega: quaternion.Vec3{X: 0.6, Y: 0.1, Z: -0.4}, Dt: 0.01},
		{Omega: quaternion.Vec3{X: -0.2, Y: 0.5, Z: 0.3}, Dt: 0.01},
		{Omega: quaternion.Vec3{X: 0.1, Y: -0.7, Z: 0.2}, Dt: 0.01},
		{Omega: quaternion.Vec3{X: 0.4, Y: 0.2, Z: 0.6}, Dt: 0.01},
	}
	cfg := Config{Method: MethodRK4, WarnNormDrift: 1}
	fwd, err := Integrate(quaternion.Identity(), base, cfg)
	if err != nil {
		t.Fatalf("fwd: %v", err)
	}
	back, err := Integrate(fwd.Final, reverseNegated(base), cfg)
	if err != nil {
		t.Fatalf("retrace: %v", err)
	}
	if d := quaternion.GeodesicDistance(back.Final, quaternion.Identity()); d > 1e-8 {
		t.Errorf("倒序反号倒放后未回到初始姿态，测地偏差 %.3e", d)
	}

	// 倒放序列单独从恒等姿态积分，应与正向全程之逆相同。
	inv, err := Integrate(quaternion.Identity(), reverseNegated(base), cfg)
	if err != nil {
		t.Fatalf("inverse run: %v", err)
	}
	if d := quaternion.GeodesicDistance(inv.Final, fwd.Final.Conj()); d > 1e-8 {
		t.Errorf("倒序反号序列与正向之逆偏差 %.3e", d)
	}
}

// 事实 3：绕固定轴转整整一圈，得到的四元数与初始姿态是同一朝向
// （允许整体差一个符号，用测地距离判定）。
func TestFullRotation_SameOrientation(t *testing.T) {
	axis := quaternion.Vec3{X: 3, Y: -4, Z: 0} // 长度 5
	axis = quaternion.ScaleVec(axis, 2*math.Pi/5)
	s := constSamples(axis, 0.01, 100) // 总时长 1s，总转角 2π

	// RK4 单步相位误差 O((ωdt)^5)，100 步累积约 2.6e-8；指数映射每步精确。
	cases := []struct {
		method string
		tol    float64
	}{
		{MethodRK4, 1e-6},
		{MethodExp, 1e-12},
	}
	for _, tc := range cases {
		res, err := Integrate(quaternion.Identity(), s, Config{Method: tc.method})
		if err != nil {
			t.Fatalf("method=%s: %v", tc.method, err)
		}
		if d := quaternion.GeodesicDistance(res.Final, quaternion.Identity()); d > tc.tol {
			t.Errorf("method=%s: 转满一圈后与初始朝向偏差 %.3e 超过 %.0e", tc.method, d, tc.tol)
		}
	}
}

// 事实 4：角速度恒为零，姿态必须冻结不动（初始姿态非平凡）。
func TestZeroOmega_Frozen(t *testing.T) {
	q0 := euler.ToQuaternion(euler.Angles{Yaw: 0.5, Pitch: 0.2, Roll: -0.3})
	s := constSamples(quaternion.Vec3{}, 0.1, 5)
	for _, method := range []string{MethodRK4, MethodExp} {
		res, err := Integrate(q0, s, Config{Method: method})
		if err != nil {
			t.Fatalf("method=%s: %v", method, err)
		}
		if d := quaternion.GeodesicDistance(res.Final, q0); d > 1e-14 {
			t.Errorf("method=%s: 零角速度下姿态漂移了 %.3e", method, d)
		}
		if res.MaxStepDrift > 1e-16 {
			t.Errorf("method=%s: 零角速度不应有范数漂移，得到 %.3e", method, res.MaxStepDrift)
		}
	}
}

// 事实 5：恒定角速度绕固定轴，转角 = |ω|·t。|ω|=0.7 rad/s，0.4s → 0.28 rad。
func TestConstantAxis_AngleEqualsOmegaTimesTime(t *testing.T) {
	w := quaternion.Vec3{X: 0.2, Y: -0.3, Z: 0.6} // |w|=0.7
	s := constSamples(w, 0.005, 80)               // 0.4 s
	wantAngle := w.Norm() * 0.4

	cases := []struct {
		method string
		tol    float64
	}{
		{MethodRK4, 1e-7},
		{MethodExp, 1e-12},
	}
	for _, tc := range cases {
		res, err := Integrate(quaternion.Identity(), s, Config{Method: tc.method})
		if err != nil {
			t.Fatalf("method=%s: %v", tc.method, err)
		}
		got := quaternion.RotationAngle(res.Final)
		if math.Abs(got-wantAngle) > tc.tol {
			t.Errorf("method=%s: 转角 %.10f 与 |ω|t=%.10f 偏差 %.3e 超过 %.0e",
				tc.method, got, wantAngle, math.Abs(got-wantAngle), tc.tol)
		}
		if math.Abs(res.FinalNorm-1) > 1e-14 {
			t.Errorf("method=%s: 末端模长偏离 1：%.3e", tc.method, res.FinalNormDrift)
		}
	}
}

// 手算核对基准：纯绕竖直轴（z/yaw）匀速 ωz=0.5 rad/s 共 1s。
// yaw 应随时间线性增长、俯仰滚转保持接近零。
func TestPureYawBenchmark_LinearYawZeroPitchRoll(t *testing.T) {
	w := quaternion.Vec3{Z: 0.5}
	s := constSamples(w, 0.01, 100) // yaw 0 → 0.5 rad
	res, err := Integrate(quaternion.Identity(), s, Config{IncludeTrace: true})
	if err != nil {
		t.Fatalf("%v", err)
	}
	end := euler.FromQuaternion(res.Final)
	if math.Abs(end.Yaw-0.5) > 1e-7 {
		t.Errorf("末端 yaw=%.10f，期望 0.5", end.Yaw)
	}
	if math.Abs(end.Pitch) > 1e-8 || math.Abs(end.Roll) > 1e-8 {
		t.Errorf("纯 yaw 旋转出现 pitch/roll：roll=%.3e pitch=%.3e", end.Roll, end.Pitch)
	}
	if end.Singular {
		t.Errorf("该姿态不在奇异区，不应报奇异")
	}
	if len(res.Trace) != 101 {
		t.Fatalf("trace 点数 = %d，期望 101", len(res.Trace))
	}

	// 逐点核对 yaw(t) ≈ 0.5t，并做最小二乘斜率检查。
	var sumT, sumY, sumTT, sumTY float64
	for k, tp := range res.Trace {
		q := quaternion.Quat{W: tp.Quat[0], X: tp.Quat[1], Y: tp.Quat[2], Z: tp.Quat[3]}
		a := euler.FromQuaternion(q)
		want := 0.5 * tp.T
		if math.Abs(a.Yaw-want) > 2e-7 {
			t.Errorf("t=%.2f 处 yaw=%.9f，期望 %.9f", tp.T, a.Yaw, want)
		}
		if math.Abs(a.Pitch) > 2e-8 || math.Abs(a.Roll) > 2e-8 {
			t.Errorf("t=%.2f 处 pitch/roll 非零：%.3e %.3e", tp.T, a.Roll, a.Pitch)
		}
		_ = k
		sumT += tp.T
		sumY += a.Yaw
		sumTT += tp.T * tp.T
		sumTY += tp.T * a.Yaw
	}
	n := float64(len(res.Trace))
	slope := (n*sumTY - sumT*sumY) / (n*sumTT - sumT*sumT)
	if math.Abs(slope-0.5) > 1e-6 {
		t.Errorf("yaw-t 最小二乘斜率 %.9f，期望 0.5", slope)
	}
}

// 大步长导致 RK4 单步范数漂移超阈值：必须拒绝而不是闷头积分。
func TestLargeStep_DriftRejected(t *testing.T) {
	s := constSamples(quaternion.Vec3{Z: 1000}, 1.0, 1)
	_, err := Integrate(quaternion.Identity(), s, Config{Method: MethodRK4})
	var rej *StepRejectedError
	if err == nil {
		t.Fatal("期望大步长被拒绝，实际成功")
	}
	if !asStepRejected(err, &rej) {
		t.Fatalf("期望 *StepRejectedError，得到 %T: %v", err, err)
	}
	if rej.Step != 1 || rej.Drift <= rej.Threshold {
		t.Errorf("拒绝信息异常: %+v", rej)
	}
}

// 中等步长：漂移超告警阈值但未达拒绝阈值 → 带告警完成。
func TestModerateStep_DriftWarned(t *testing.T) {
	// 单步转角 ωdt=0.8 rad（x=0.4），RK4 漂移约 x^6/144 ≈ 2.8e-5。
	s := constSamples(quaternion.Vec3{Z: 40}, 0.02, 2)
	res, err := Integrate(quaternion.Identity(), s, Config{Method: MethodRK4})
	if err != nil {
		t.Fatalf("不应被拒绝：%v", err)
	}
	if len(res.Warnings) == 0 {
		t.Fatalf("期望出现范数漂移告警，实际没有；max drift=%.3e", res.MaxStepDrift)
	}
	if res.MaxStepDrift <= 1e-6 {
		t.Errorf("期望最大漂移 >1e-6，得到 %.3e", res.MaxStepDrift)
	}
}

// 两种推进方法在正常步长下应高度一致。
func TestRK4AndExpAgree(t *testing.T) {
	s := []Sample{
		{Omega: quaternion.Vec3{X: 0.3, Y: -0.2, Z: 0.5}, Dt: 0.01},
		{Omega: quaternion.Vec3{X: -0.4, Y: 0.1, Z: 0.2}, Dt: 0.01},
		{Omega: quaternion.Vec3{X: 0.15, Y: 0.6, Z: -0.3}, Dt: 0.01},
	}
	a, err := Integrate(quaternion.Identity(), s, Config{Method: MethodRK4, WarnNormDrift: 1})
	if err != nil {
		t.Fatal(err)
	}
	b, err := Integrate(quaternion.Identity(), s, Config{Method: MethodExp, WarnNormDrift: 1})
	if err != nil {
		t.Fatal(err)
	}
	if d := quaternion.GeodesicDistance(a.Final, b.Final); d > 1e-9 {
		t.Errorf("RK4 与指数映射结果偏差 %.3e", d)
	}
}

// 两段序列并发推进，彼此的积分状态必须完全隔离。
func TestConcurrentRuns_AreIsolated(t *testing.T) {
	const workers = 32
	var wg sync.WaitGroup
	errs := make(chan error, workers)
	wg.Add(workers)
	for g := 0; g < workers; g++ {
		g := g
		go func() {
			defer wg.Done()
			// 偶数 goroutine 走 yaw 匀速，奇数走另一段多轴序列，
			// 各自都有独立的 Result/Trace，不共享任何包级状态。
			if g%2 == 0 {
				res, err := Integrate(quaternion.Identity(),
					constSamples(quaternion.Vec3{Z: 0.5}, 0.01, 100),
					Config{IncludeTrace: true})
				if err != nil {
					errs <- err
					return
				}
				if math.Abs(euler.FromQuaternion(res.Final).Yaw-0.5) > 1e-7 {
					errs <- errMismatch("yaw worker", res.Final)
					return
				}
			} else {
				res, err := Integrate(quaternion.Identity(),
					constSamples(quaternion.Vec3{X: 0.7, Y: -0.4, Z: 0.2}, 0.01, 70),
					Config{IncludeTrace: true})
				if err != nil {
					errs <- err
					return
				}
				want := quaternion.Vec3{X: 0.7, Y: -0.4, Z: 0.2}
				if ang := quaternion.RotationAngle(res.Final); math.Abs(ang-want.Norm()*0.7) > 1e-7 {
					errs <- errMismatch("multi-axis worker", res.Final)
					return
				}
			}
		}()
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}

func asStepRejected(err error, target **StepRejectedError) bool {
	e, ok := err.(*StepRejectedError)
	*target = e
	return ok
}

func errMismatch(where string, q quaternion.Quat) error {
	return fmt.Errorf("%s: 末端姿态与期望值不符 [%.6f %.6f %.6f %.6f]",
		where, q.W, q.X, q.Y, q.Z)
}
