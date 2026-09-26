# syntax=docker/dockerfile:1

# ---- 构建阶段 ----
FROM golang:1.22-alpine AS build
WORKDIR /src

# 先拷 go.mod/go.sum 以利用依赖缓存
COPY go.mod go.sum ./
RUN go mod download

COPY . .
RUN CGO_ENABLED=0 go test ./... \
 && CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" \
      -o /out/attitude-server ./cmd/attitude-server

# ---- 运行阶段 ----
FROM alpine:3.20
RUN apk add --no-cache ca-certificates \
 && adduser -D -u 10001 attitude
USER attitude

COPY --from=build /out/attitude-server /usr/local/bin/attitude-server

ENV ATTITUDE_HTTP_ADDR=:8080
EXPOSE 8080

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s \
  CMD wget -qO- http://127.0.0.1:8080/healthz >/dev/null || exit 1

ENTRYPOINT ["attitude-server"]
