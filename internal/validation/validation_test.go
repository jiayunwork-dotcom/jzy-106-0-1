package validation

import (
	"errors"
	"math"
	"testing"

	"attitude-service/internal/integrator"
)

var nan = math.NaN()

func goodRaw() RawInput {
	return RawInput{
		Q0:     [4]float64{1, 0, 0, 0},
		Omegas: [][3]float64{{0, 0, 0.1}, {0, 0, 0.2}},
		Dts:    []float64{0.1, 0.1},
	}
}

func TestValidateOK(t *testing.T) {
	in, err := Validate(goodRaw())
	if err != nil {
		t.Fatalf("合法输入被拒：%v", err)
	}
	if in.Mode != ModeDts {
		t.Errorf("mode=%s", in.Mode)
	}
	if len(in.Steps) != 2 || in.Steps[1].Omega.Z != 0.2 || in.Steps[1].Dt != 0.1 {
		t.Errorf("steps 转换异常：%+v", in.Steps)
	}
	if in.Config.Method != integrator.MethodRK4 {
		t.Errorf("默认方法应为 rk4，得到 %q", in.Config.Method)
	}
	if in.Config.WarnNormDrift != 1e-6 || in.Config.RejectNormDrift != 1e-2 {
		t.Errorf("默认阈值异常：%+v", in.Config)
	}
}

func TestValidateTimestamps(t *testing.T) {
	raw := goodRaw()
	raw.Dts = nil
	raw.Timestamps = []float64{0, 0.1, 0.25}
	in, err := Validate(raw)
	if err != nil {
		t.Fatalf("合法时间戳输入被拒：%v", err)
	}
	if in.Mode != ModeTimestamps || in.Steps[1].Dt != 0.15 {
		t.Errorf("timestamps 模式 dt 推算错误：%+v", in.Steps)
	}
}

func TestValidateErrors(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(*RawInput)
		field  string
	}{
		{
			"空序列",
			func(r *RawInput) { r.Omegas = nil; r.Dts = nil },
			"angular_velocities",
		},
		{
			"初始四元数非单位",
			func(r *RawInput) { r.Q0 = [4]float64{1, 1, 0, 0} },
			"initial_quaternion",
		},
		{
			"初始四元数含 NaN",
			func(r *RawInput) { r.Q0 = [4]float64{1, nan, 0, 0} },
			"initial_quaternion",
		},
		{
			"时间步长非正",
			func(r *RawInput) { r.Dts[0] = 0 },
			"time_steps[0]",
		},
		{
			"时间步长为负",
			func(r *RawInput) { r.Dts[1] = -0.01 },
			"time_steps[1]",
		},
		{
			"时间戳个数不匹配",
			func(r *RawInput) { r.Dts = nil; r.Timestamps = []float64{0, 0.1} },
			"timestamps",
		},
		{
			"dts 个数不匹配",
			func(r *RawInput) { r.Dts = []float64{0.1} },
			"time_steps",
		},
		{
			"时间戳非递增",
			func(r *RawInput) { r.Dts = nil; r.Timestamps = []float64{0, 0.2, 0.1} },
			"timestamps[2]",
		},
		{
			"角速度含 NaN",
			func(r *RawInput) { r.Omegas[1] = [3]float64{0, nan, 0} },
			"angular_velocities[1]",
		},
		{
			"未知方法",
			func(r *RawInput) { r.Method = "euler" },
			"method",
		},
		{
			"告警阈值大于拒绝阈值",
			func(r *RawInput) { r.WarnDrift = 0.1; r.RejectDrift = 1e-3 },
			"warn_norm_drift",
		},
		{
			"同时给时间戳和步长",
			func(r *RawInput) { r.Timestamps = []float64{0, 0.1, 0.2} },
			"timestamps",
		},
	}
	for _, tc := range cases {
		raw := goodRaw()
		tc.mutate(&raw)
		_, err := Validate(raw)
		if err == nil {
			t.Errorf("%s：期望被拒，实际通过", tc.name)
			continue
		}
		var fe *FieldError
		if !errors.As(err, &fe) {
			t.Errorf("%s：期望 *FieldError，得到 %T %v", tc.name, err, err)
			continue
		}
		if fe.Field != tc.field {
			t.Errorf("%s：错误字段 %q，期望 %q；原因 %s", tc.name, fe.Field, tc.field, fe.Reason)
		}
		if fe.Reason == "" {
			t.Errorf("%s：错误原因为空", tc.name)
		}
	}
}

func TestValidateNoTimeInfo(t *testing.T) {
	raw := goodRaw()
	raw.Dts = nil
	if _, err := Validate(raw); err == nil {
		t.Error("缺少时间信息应被拒")
	}
}

func TestNameRule(t *testing.T) {
	for _, good := range []string{"a", "bench-1", "yaw_case_02", "ABC123_-"} {
		if err := NameRule(good); err != nil {
			t.Errorf("名字 %q 应合法：%v", good, err)
		}
	}
	for _, bad := range []string{"", "a/b", "a b", "中文", "a.b"} {
		if err := NameRule(bad); err == nil {
			t.Errorf("名字 %q 应非法", bad)
		}
	}
	if err := NameRule(string(make([]byte, 65))); err == nil {
		t.Error("超过 64 字节的名字应非法")
	}
}
