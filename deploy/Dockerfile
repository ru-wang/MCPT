FROM archlinux:latest

RUN echo 'Server = http://mirrors.tuna.tsinghua.edu.cn/archlinux/$repo/os/$arch' > /etc/pacman.d/mirrorlist \
    && pacman -Syu --noconfirm openssh gcc cmake ninja git eigen glfw-x11 glew \
    && pacman -Scc --noconfirm

RUN useradd -m drinker
USER drinker
WORKDIR /home/drinker
