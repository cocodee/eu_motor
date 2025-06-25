# 定义你的Docker镜像名称
IMAGE_NAME="eu_motor_base:latest"

# 运行容器，并将当前目录映射到/app
# -it:  以交互模式运行，并分配一个伪终端 (方便调试)
# --rm: 容器退出后自动删除 (保持环境干净)
# -v:   挂载卷，格式是 <host-path>:<container-path>
# ${PWD}:  这是一个Shell变量，代表“Print Working Directory”，即当前目录
docker run -it --rm -v "${PWD}:/app" ${IMAGE_NAME}