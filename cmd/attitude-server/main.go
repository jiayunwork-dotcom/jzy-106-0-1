// Command attitude-server 启动刚体姿态四元数积分 HTTP 服务。
// 只提供姿态本身的积分接口，不涉及位置/速度/惯导滤波，也没有任何页面。
package main

import (
	"log"
	"os"

	"attitude-service/internal/api"
	"attitude-service/internal/store"
)

func main() {
	addr := os.Getenv("ATTITUDE_HTTP_ADDR")
	if addr == "" {
		if p := os.Getenv("PORT"); p != "" {
			addr = ":" + p
		} else {
			addr = ":8080"
		}
	}

	r := api.NewRouter(store.NewMemoryStore())
	log.Printf("attitude integration service listening on %s", addr)
	if err := r.Run(addr); err != nil {
		log.Fatalf("server failed: %v", err)
	}
}
