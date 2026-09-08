# twitch_gimbal Linux 环境与移植指南

本工程面向 `STM32F405RGT6`，使用 GNU Arm Embedded GCC、CMake 和 Ninja 构建，
通过 SWD 使用 J-Link 或 CMSIS-DAP/DAPLink 下载调试。本文记录了在 Ubuntu
22.04 上验证通过的环境配置方法。环境适配只发生在本机，不需要修改工程中的
`CMakeLists.txt`、`CMakePresets.json`、Makefile、链接脚本或调试配置。

## 1. 已验证环境

| 组件 | 验证版本 | 用途 |
| --- | --- | --- |
| GNU Tools for STM32 | 14.3.1+st.2 | C/C++/ASM 交叉编译、objcopy、size、GDB |
| CMake | 4.3.1+st.1 | 读取工程 preset 并生成 Ninja 构建文件 |
| Ninja | 1.13.2+st.1 | 实际执行编译 |
| GNU Make | 4.3 | 执行工程 Makefile 中的封装目标 |
| OpenOCD | 0.11.0 | J-Link 或 CMSIS-DAP 下载调试 |
| SEGGER J-Link Software | 9.68 | J-Link Commander 和 GDB Server |
| pyOCD | 0.45.1 | CMSIS-DAP/DAPLink 枚举、下载和调试 |
| Keil STM32F4xx DFP | 3.1.1 | pyOCD 的 STM32F405 目标描述 |

工程要求 C11、C++17，目标参数为 Cortex-M4F、单精度 FPU、硬浮点 ABI：

```text
-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
```

## 2. 安装基础依赖

Ubuntu 22.04 可先安装基础命令和 USB 支持：

```bash
sudo apt update
sudo apt install make cmake ninja-build gcc-arm-none-eabi \
  libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
  gdb-multiarch openocd libusb-1.0-0 python3-pip
sudo usermod -aG plugdev "$USER"
```

重新登录后 `plugdev` 组才会在所有终端中生效。Ubuntu 仓库中的 GCC 版本可以
构建本工程，但为了复现本次验证结果，建议按下一节使用工程元数据中记录的 ST
bundle 版本。

## 3. 接入工程指定的 ST 工具链

安装 VS Code 的 STM32Cube 扩展并打开工程后，让扩展下载 `.settings` 中声明的
三个 bundle：

```text
cmake                 4.3.1+st.1
ninja                 1.13.2+st.1
gnu-tools-for-stm32   14.3.1+st.2
```

Linux 下默认下载目录为 `~/.local/share/stm32cube/bundles`。为保证普通终端、
Makefile 和 VS Code 使用同一套工具，可建立用户级命令链接：

```bash
STM32_BUNDLES="${HOME}/.local/share/stm32cube/bundles"
LOCAL_COMMANDS="${HOME}/.local/bin"
CMAKE_BUNDLE="${STM32_BUNDLES}/cmake/4.3.1+st.1/bin"
NINJA_BUNDLE="${STM32_BUNDLES}/ninja/1.13.2+st.1/bin"
GCC_BUNDLE="${STM32_BUNDLES}/gnu-tools-for-stm32/14.3.1+st.2/bin"

mkdir -p "${LOCAL_COMMANDS}"
ln -sfn "${CMAKE_BUNDLE}/cmake" "${LOCAL_COMMANDS}/cmake"
ln -sfn "${CMAKE_BUNDLE}/cmake" "${LOCAL_COMMANDS}/cube-cmake"
ln -sfn "${CMAKE_BUNDLE}/ctest" "${LOCAL_COMMANDS}/ctest"
ln -sfn "${CMAKE_BUNDLE}/cpack" "${LOCAL_COMMANDS}/cpack"
ln -sfn "${NINJA_BUNDLE}/ninja" "${LOCAL_COMMANDS}/ninja"

for STM32_TOOL in "${GCC_BUNDLE}"/arm-none-eabi-*; do
  ln -sfn "${STM32_TOOL}" "${LOCAL_COMMANDS}/$(basename "${STM32_TOOL}")"
done
```

确认 `~/.local/bin` 位于 PATH 前部；Ubuntu 的默认用户配置通常已包含该目录。
若 VS Code 或桌面会话是在安装工具前启动的，需要将该目录加入交互式 shell 配置，
并重载当前终端：

```bash
grep -qF '$HOME/.local/bin' "${HOME}/.bashrc" || \
  printf '\nexport PATH="$HOME/.local/bin:$PATH"\n' >> "${HOME}/.bashrc"
source "${HOME}/.bashrc"
hash -r
```

重新打开终端后检查版本：

```bash
command -v cmake cube-cmake ninja arm-none-eabi-gcc
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

若 `make` 报错 `make: cmake: 没有那个文件或目录`，先执行：

```bash
export PATH="${HOME}/.local/bin:${PATH}"
hash -r
command -v cmake
```

正常结果应指向 `~/.local/bin/cmake`。如果 VS Code 集成终端仍保留旧 PATH，完全
退出并重新启动 VS Code。也可以在 VS Code 用户设置（不是工程设置）中加入：

```json
"terminal.integrated.env.linux": {
    "PATH": "${userHome}/.local/bin:${env:PATH}"
}
```

修改该设置后必须新建一个集成终端；已经打开的终端进程不会自动改变环境变量。

这里保留了工程原有的 GCC 工具链类型，没有改用 Clang、ARM Compiler 或其他
构建系统。

## 4. J-Link 安装

从 SEGGER 官方 J-Link 下载页面阅读并接受许可条款，下载当前 x86_64 Debian
软件包。具备 sudo 权限时，建议按官方方式系统安装：

```bash
sudo apt install ./JLink_Linux_V968_x86_64.deb
sudo udevadm control --reload-rules
sudo udevadm trigger
```

不能在非交互终端输入 sudo 密码时，可以将同一官方包安装到用户目录：

```bash
JLINK_USER_ROOT="${HOME}/.local/share/segger/jlink-9.68.0"
JLINK_COMMANDS="${HOME}/.local/bin"

mkdir -p "${JLINK_USER_ROOT}" "${JLINK_COMMANDS}"
dpkg-deb -x ./JLink_Linux_V968_x86_64.deb "${JLINK_USER_ROOT}"
JLINK_BIN="${JLINK_USER_ROOT}/opt/SEGGER/JLink_V968"

for JLINK_TOOL in "${JLINK_BIN}"/*Exe; do
  ln -sfn "${JLINK_TOOL}" "${JLINK_COMMANDS}/$(basename "${JLINK_TOOL}")"
done
ln -sfn "${JLINK_BIN}/JLinkExe" "${JLINK_COMMANDS}/JLink"
ln -sfn "${JLINK_BIN}/JLinkGDBServerCLExe" \
  "${JLINK_COMMANDS}/JLinkGDBServer"
```

用户级安装不会写入 `/etc/udev/rules.d`。本机已有 OpenOCD 提供的
`/lib/udev/rules.d/60-openocd.rules`，其中覆盖 SEGGER USB VID `1366`，因此
用户级 J-Link 工具仍可访问探针。新机器应检查：

```bash
grep -n '1366' /lib/udev/rules.d/60-openocd.rules
id -nG | grep -w plugdev
JLinkExe
```

工程的 `openocd.cfg` 已选择 `interface/jlink.cfg`、SWD 和 STM32F4 目标，接好
J-Link 后可以直接执行：

```bash
make flash
```

`twitch_gimbal_debug.jdebug` 中记录了原开发者的 J-Link 序列号。其他探针可优先
使用不绑定序列号的 OpenOCD 流程，避免为了本机设备修改并提交工程调试文件。

## 5. DAPLink/CMSIS-DAP 安装

DAPLink 在 Linux 上使用内核自带的 HID/USB 支持，不需要安装 Windows 风格的
设备驱动。主机侧需要 udev 权限和 OpenOCD 或 pyOCD。OpenOCD 的规则文件应含有：

```text
ATTRS{product}=="*CMSIS-DAP*", MODE="660", GROUP="plugdev", TAG+="uaccess"
```

安装 pyOCD，并下载 STM32F405 设备包：

```bash
python3 -m pip install --user --upgrade pyocd
pyocd pack install stm32f405rg
pyocd list --targets | grep -i stm32f405rg
pyocd list --probes
```

未插入探针时 `pyocd list --probes` 输出 `No available debug probes are
connected` 属于正常结果。插入 DAPLink 后可使用以下 OpenOCD 配置启动调试服务：

```bash
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg
```

或者使用 pyOCD：

```bash
pyocd gdbserver --target stm32f405rg
```

## 6. 构建工程

仓库中的 `build/Debug` 曾包含其他路径和旧编译器生成的缓存。首次在新路径构建
时必须刷新缓存，否则 CMake 可能报告源码目录不一致：

```bash
cmake --fresh --preset Debug
make artifacts
```

生成文件位于：

```text
build/Debug/twitch_gimbal.elf
build/Debug/twitch_gimbal.hex
build/Debug/twitch_gimbal.bin
```

Release 构建：

```bash
cmake --fresh --preset Release
make release
```

生成文件位于 `build/Release`。本次 GCC 14.3.1 验证结果为：

| 配置 | FLASH | RAM | 结果 |
| --- | ---: | ---: | --- |
| Debug | 68,344 B | 26,792 B | 成功 |
| Release | 37,388 B | 26,528 B | 成功 |

检查产物确实由预期编译器生成：

```bash
arm-none-eabi-readelf -p .comment build/Debug/twitch_gimbal.elf
arm-none-eabi-size build/Debug/twitch_gimbal.elf
```

工程当前存在初始化顺序、未使用变量、格式字符串、潜在未初始化变量和优化循环
越界等编译警告，但不会阻止链接。这些属于源码质量问题，不应通过更换工具链或
降低工程编译设置来掩盖；环境移植时保持原配置即可。

## 7. 移植验收清单

1. `cmake`、`ninja` 和 `arm-none-eabi-gcc` 指向预期 ST bundle。
2. `cmake --fresh --preset Debug` 成功识别 C、C++ 和 ASM 编译器。
3. `make artifacts` 和 `make release` 均生成 ELF、HEX、BIN。
4. `JLinkExe` 能启动并显示版本，插入探针后能够枚举设备。
5. `pyocd list --targets` 包含 `stm32f405rg`。
6. 插入 DAPLink 后 `pyocd list --probes` 能显示探针。
7. 实际烧录前确认目标板供电、SWDIO、SWCLK、GND 和复位线连接正确。
8. 构建生成物不要作为工具链迁移修改提交；只提交有意修改的源码和配置。
