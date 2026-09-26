package api

import (
	"bytes"
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"

	"github.com/gin-gonic/gin"

	"attitude-service/internal/store"
)

func newServer(t *testing.T) *httptest.Server {
	t.Helper()
	gin.SetMode(gin.TestMode)
	srv := httptest.NewServer(NewRouter(store.NewMemoryStore()))
	t.Cleanup(srv.Close)
	return srv
}

func post(t *testing.T, url string, body any) (int, map[string]any) {
	t.Helper()
	buf, err := json.Marshal(body)
	if err != nil {
		t.Fatal(err)
	}
	resp, err := http.Post(url, "application/json", bytes.NewReader(buf))
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	var out map[string]any
	if err := json.NewDecoder(resp.Body).Decode(&out); err != nil {
		t.Fatalf("响应不是 JSON: %v", err)
	}
	return resp.StatusCode, out
}

func get(t *testing.T, url string) (int, map[string]any) {
	t.Helper()
	resp, err := http.Get(url)
	if err != nil {
		t.Fatal(err)
	}
	defer resp.Body.Close()
	var out map[string]any
	if err := json.NewDecoder(resp.Body).Decode(&out); err != nil {
		t.Fatalf("响应不是 JSON: %v", err)
	}
	return resp.StatusCode, out
}

func del(t *testing.T, url string) int {
	t.Helper()
	req, _ := http.NewRequest(http.MethodDelete, url, nil)
	resp, err := http.DefaultClient.Do(req)
	if err != nil {
		t.Fatal(err)
	}
	resp.Body.Close()
	return resp.StatusCode
}

func num(t *testing.T, m map[string]any, key string) float64 {
	t.Helper()
	v, ok := m[key].(float64)
	if !ok {
		t.Fatalf("字段 %q 缺失或非数字：%v", key, m[key])
	}
	return v
}

// 匀速绕 z 轴 0.5 rad/s × 1s：末端 yaw 应 ≈ 0.5。
func yawRequest() map[string]any {
	omegas := make([][]float64, 100)
	dts := make([]float64, 100)
	for i := range omegas {
		omegas[i] = []float64{0, 0, 0.5}
		dts[i] = 0.01
	}
	return map[string]any{
		"initial_quaternion": []float64{1, 0, 0, 0},
		"angular_velocities": omegas,
		"time_steps":         dts,
	}
}

func TestIntegratePureYawBenchmark(t *testing.T) {
	srv := newServer(t)
	code, body := post(t, srv.URL+"/api/v1/attitude/integrate", yawRequest())
	if code != http.StatusOK {
		t.Fatalf("状态码 %d: %v", code, body)
	}
	e := body["final_euler"].(map[string]any)
	if got := num(t, e, "yaw"); math.Abs(got-0.5) > 1e-7 {
		t.Errorf("yaw=%.9f，期望 0.5", got)
	}
	if math.Abs(num(t, e, "pitch")) > 1e-8 || math.Abs(num(t, e, "roll")) > 1e-8 {
		t.Errorf("pitch/roll 应接近 0：%v", e)
	}
	if e["convention"] != "ZYX" {
		t.Errorf("convention=%v", e["convention"])
	}
	if e["singular"].(bool) {
		t.Error("不应奇异")
	}
	if d := num(t, body, "final_norm_drift"); d > 1e-14 {
		t.Errorf("末端范数漂移 %.3e", d)
	}
	if q := body["final_quaternion"].([]any); len(q) != 4 {
		t.Errorf("final_quaternion 长度 %d", len(q))
	}
}

// 时间戳模式：N 个角速度配 N+1 个时间戳。
func TestIntegrateWithTimestamps(t *testing.T) {
	srv := newServer(t)
	omegas := make([][]float64, 10)
	ts := make([]float64, 11)
	for i := range omegas {
		omegas[i] = []float64{0, 0, 0.5}
	}
	for i := range ts {
		ts[i] = float64(i) * 0.1
	}
	req := map[string]any{
		"initial_quaternion": []float64{1, 0, 0, 0},
		"angular_velocities": omegas,
		"timestamps":         ts,
	}
	code, body := post(t, srv.URL+"/api/v1/attitude/integrate", req)
	if code != http.StatusOK {
		t.Fatalf("状态码 %d: %v", code, body)
	}
	if got := num(t, body["final_euler"].(map[string]any), "yaw"); math.Abs(got-0.5) > 1e-7 {
		t.Errorf("yaw=%.9f，期望 0.5", got)
	}
	if d := num(t, body, "duration"); math.Abs(d-1.0) > 1e-12 {
		t.Errorf("duration=%.6f", d)
	}
}

func TestIntegrateRejectsBadInput(t *testing.T) {
	srv := newServer(t)
	url := srv.URL + "/api/v1/attitude/integrate"

	cases := []struct {
		name string
		body map[string]any
		want string // 错误信息应包含的关键字
	}{
		{
			"非单位初始四元数",
			map[string]any{
				"initial_quaternion": []float64{1, 1, 0, 0},
				"angular_velocities": [][]float64{{0, 0, 1}},
				"time_steps":         []float64{0.1},
			},
			"单位四元数",
		},
		{
			"空序列",
			map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": [][]float64{},
				"time_steps":         []float64{},
			},
			"为空",
		},
		{
			"时间步长非正",
			map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": [][]float64{{0, 0, 1}},
				"time_steps":         []float64{0},
			},
			"必须为正",
		},
		{
			"角速度与时间戳个数不匹配",
			map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": [][]float64{{0, 0, 1}, {0, 0, 1}},
				"timestamps":         []float64{0, 0.1},
			},
			"多 1",
		},
		{
			"角速度与步长个数不匹配",
			map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": [][]float64{{0, 0, 1}, {0, 0, 1}},
				"time_steps":         []float64{0.1},
			},
			"一致",
		},
		{
			"角速度向量长度不为 3",
			map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": [][]float64{{0, 0}},
				"time_steps":         []float64{0.1},
			},
			"长度 3",
		},
		{
			"未知积分方法",
			map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": [][]float64{{0, 0, 1}},
				"time_steps":         []float64{0.1},
				"method":             "rk45",
			},
			"rk4",
		},
		{
			"时间戳与步长同时提供",
			map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": [][]float64{{0, 0, 1}},
				"time_steps":         []float64{0.1},
				"timestamps":         []float64{0, 0.1},
			},
			"二选一",
		},
	}
	for _, tc := range cases {
		code, body := post(t, url, tc.body)
		if code != http.StatusBadRequest {
			t.Errorf("%s：状态码 %d，期望 400：%v", tc.name, code, body)
			continue
		}
		errObj, ok := body["error"].(map[string]any)
		if !ok {
			t.Errorf("%s：响应缺少 error 字段：%v", tc.name, body)
			continue
		}
		msg, _ := errObj["message"].(string)
		if msg == "" {
			t.Errorf("%s：错误原因为空", tc.name)
		}
		if !bytes.Contains([]byte(msg), []byte(tc.want)) {
			t.Errorf("%s：错误信息 %q 未包含 %q", tc.name, msg, tc.want)
		}
	}
}

// 大步长导致单步范数漂移超阈值：必须 422 拒绝。
func TestIntegrateRejectsHugeStep(t *testing.T) {
	srv := newServer(t)
	code, body := post(t, srv.URL+"/api/v1/attitude/integrate", map[string]any{
		"initial_quaternion": []float64{1, 0, 0, 0},
		"angular_velocities": [][]float64{{0, 0, 1000}},
		"time_steps":         []float64{1.0},
	})
	if code != http.StatusUnprocessableEntity {
		t.Fatalf("状态码 %d，期望 422：%v", code, body)
	}
	errObj := body["error"].(map[string]any)
	if errObj["code"] != "STEP_NORM_DRIFT_EXCEEDED" {
		t.Errorf("错误码 %v", errObj["code"])
	}
}

// 中等步长：完成积分但带漂移告警。
func TestIntegrateWarnsOnModerateDrift(t *testing.T) {
	srv := newServer(t)
	code, body := post(t, srv.URL+"/api/v1/attitude/integrate", map[string]any{
		"initial_quaternion": []float64{1, 0, 0, 0},
		"angular_velocities": [][]float64{{0, 0, 40}},
		"time_steps":         []float64{0.02},
	})
	if code != http.StatusOK {
		t.Fatalf("状态码 %d：%v", code, body)
	}
	warnings, ok := body["warnings"].([]any)
	if !ok || len(warnings) == 0 {
		t.Fatalf("期望告警，得到 %v", body["warnings"])
	}
}

// 俯仰 +90°：响应必须带 singular=true 与告警文字，且欧拉角为有限值。
func TestIntegrateSingularAttitude(t *testing.T) {
	srv := newServer(t)
	code, body := post(t, srv.URL+"/api/v1/attitude/integrate", map[string]any{
		"initial_quaternion": []float64{1, 0, 0, 0},
		"angular_velocities": [][]float64{{0, math.Pi / 2, 0}},
		"time_steps":         []float64{1.0},
		"method":             "exp",
	})
	if code != http.StatusOK {
		t.Fatalf("状态码 %d：%v", code, body)
	}
	e := body["final_euler"].(map[string]any)
	if !e["singular"].(bool) {
		t.Error("俯仰 90° 必须标记 singular")
	}
	if e["warning"] == nil || e["warning"] == "" {
		t.Error("必须给出奇异告警文字")
	}
	for _, k := range []string{"roll", "pitch", "yaw"} {
		v := num(t, e, k)
		if math.IsNaN(v) || math.IsInf(v, 0) {
			t.Errorf("%s 非有限值", k)
		}
	}
	if math.Abs(num(t, e, "pitch")-math.Pi/2) > 1e-9 {
		t.Errorf("pitch 应为 π/2，得到 %.9f", num(t, e, "pitch"))
	}
}

// 命名序列：保存 → 列表 → 详情 → 命名积分 → 删除。
func TestNamedSequenceLifecycle(t *testing.T) {
	srv := newServer(t)

	// 保存
	code, body := post(t, srv.URL+"/api/v1/sequences?name=bench-yaw", yawRequest())
	if code != http.StatusCreated {
		t.Fatalf("保存状态码 %d：%v", code, body)
	}
	if body["name"] != "bench-yaw" || num(t, body, "num_samples") != 100 {
		t.Errorf("保存响应异常：%v", body)
	}

	// 列表
	code, body = get(t, srv.URL+"/api/v1/sequences")
	if code != http.StatusOK {
		t.Fatalf("列表状态码 %d", code)
	}
	seqs := body["sequences"].([]any)
	if len(seqs) != 1 || seqs[0].(map[string]any)["name"] != "bench-yaw" {
		t.Errorf("列表异常：%v", body)
	}

	// 详情
	code, body = get(t, srv.URL+"/api/v1/sequences/bench-yaw")
	if code != http.StatusOK {
		t.Fatalf("详情状态码 %d", code)
	}
	if n := len(body["angular_velocities"].([]any)); n != 100 {
		t.Errorf("详情采样数 %d", n)
	}

	// 命名积分：与直接积分结果一致
	code, named := post(t, srv.URL+"/api/v1/sequences/bench-yaw/integrate",
		map[string]any{"initial_quaternion": []float64{1, 0, 0, 0}})
	if code != http.StatusOK {
		t.Fatalf("命名积分状态码 %d：%v", code, named)
	}
	code, direct := post(t, srv.URL+"/api/v1/attitude/integrate", yawRequest())
	if code != http.StatusOK {
		t.Fatalf("直接积分状态码 %d", code)
	}
	qn := named["final_quaternion"].([]any)
	qd := direct["final_quaternion"].([]any)
	for i := 0; i < 4; i++ {
		if math.Abs(qn[i].(float64)-qd[i].(float64)) > 1e-15 {
			t.Errorf("命名积分与直接积分结果不一致：%v vs %v", qn, qd)
			break
		}
	}

	// 非法名字
	code, _ = post(t, srv.URL+"/api/v1/sequences?name=bad/name", yawRequest())
	if code != http.StatusBadRequest {
		t.Errorf("非法名字应 400，得到 %d", code)
	}

	// 不存在的序列
	code, body = get(t, srv.URL+"/api/v1/sequences/nope")
	if code != http.StatusNotFound {
		t.Errorf("不存在序列应 404，得到 %d：%v", code, body)
	}

	// 删除
	if code := del(t, srv.URL+"/api/v1/sequences/bench-yaw"); code != http.StatusNoContent {
		t.Errorf("删除应 204，得到 %d", code)
	}
	if code, _ := get(t, srv.URL+"/api/v1/sequences/bench-yaw"); code != http.StatusNotFound {
		t.Errorf("删除后应 404，得到 %d", code)
	}
}

// include_trace=true 时返回每个时间点的姿态，yaw 应线性增长。
func TestIntegrateTrace(t *testing.T) {
	srv := newServer(t)
	req := yawRequest()
	req["include_trace"] = true
	code, body := post(t, srv.URL+"/api/v1/attitude/integrate", req)
	if code != http.StatusOK {
		t.Fatalf("状态码 %d：%v", code, body)
	}
	trace, ok := body["trace"].([]any)
	if !ok || len(trace) != 101 {
		t.Fatalf("trace 应 101 点，得到 %v", body["trace"])
	}
	for i, tp := range trace {
		p := tp.(map[string]any)
		yaw := p["euler"].(map[string]any)["yaw"].(float64)
		want := 0.5 * p["t"].(float64)
		if math.Abs(yaw-want) > 2e-7 {
			t.Errorf("trace[%d] t=%.2f yaw=%.9f 期望 %.9f", i, p["t"], yaw, want)
		}
	}
}

// 并发请求：两段不同序列同时推进，结果必须各自正确、互不渗透。
func TestConcurrentIntegrationsIsolated(t *testing.T) {
	srv := newServer(t)
	const workers = 16
	var wg sync.WaitGroup
	errs := make(chan error, workers*2)
	wg.Add(workers * 2)
	for g := 0; g < workers; g++ {
		go func() {
			defer wg.Done()
			code, body := post(t, srv.URL+"/api/v1/attitude/integrate", yawRequest())
			if code != http.StatusOK {
				errs <- fmt.Errorf("yaw 请求状态码 %d", code)
				return
			}
			yaw := body["final_euler"].(map[string]any)["yaw"].(float64)
			if math.Abs(yaw-0.5) > 1e-7 {
				errs <- fmt.Errorf("yaw 请求结果 %.9f", yaw)
			}
		}()
		go func() {
			defer wg.Done()
			// 绕 x 轴 0.3 rad/s × 2s → roll 0.6
			omegas := make([][]float64, 200)
			dts := make([]float64, 200)
			for i := range omegas {
				omegas[i] = []float64{0.3, 0, 0}
				dts[i] = 0.01
			}
			code, body := post(t, srv.URL+"/api/v1/attitude/integrate", map[string]any{
				"initial_quaternion": []float64{1, 0, 0, 0},
				"angular_velocities": omegas,
				"time_steps":         dts,
			})
			if code != http.StatusOK {
				errs <- fmt.Errorf("roll 请求状态码 %d", code)
				return
			}
			roll := body["final_euler"].(map[string]any)["roll"].(float64)
			if math.Abs(roll-0.6) > 1e-7 {
				errs <- fmt.Errorf("roll 请求结果 %.9f", roll)
			}
		}()
	}
	wg.Wait()
	close(errs)
	for err := range errs {
		t.Error(err)
	}
}

func TestHealth(t *testing.T) {
	srv := newServer(t)
	code, body := get(t, srv.URL+"/healthz")
	if code != http.StatusOK || body["status"] != "ok" {
		t.Errorf("healthz 异常：%d %v", code, body)
	}
}
