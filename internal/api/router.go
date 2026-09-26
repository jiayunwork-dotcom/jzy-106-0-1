package api

import (
	"github.com/gin-gonic/gin"

	"attitude-service/internal/store"
)

// NewRouter 组装全部 HTTP 路由。接口层只有姿态积分相关接口，
// 没有任何页面。
func NewRouter(seqs store.Store) *gin.Engine {
	r := gin.New()
	r.Use(gin.Logger(), gin.Recovery())

	h := NewHandler(seqs)

	r.GET("/healthz", h.Health)

	v1 := r.Group("/api/v1")
	{
		// 直接推进一段角速度序列。
		v1.POST("/attitude/integrate", h.Integrate)

		// 命名角速度序列：存取与复用。
		v1.POST("/sequences", h.SaveSequence)
		v1.GET("/sequences", h.ListSequences)
		v1.GET("/sequences/:name", h.GetSequence)
		v1.DELETE("/sequences/:name", h.DeleteSequence)
		v1.POST("/sequences/:name/integrate", h.IntegrateSequence)
	}
	return r
}
