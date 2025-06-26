Import("env")
import os
import subprocess

# Paths
bootloader_path = os.path.abspath(os.path.join(env.subst("$PROJECT_DIR"), "MicroNodeBootloader.bin")).replace("\\", "/")
firmware_path = os.path.abspath(os.path.join(env.subst("$BUILD_DIR"), "firmware.bin")).replace("\\", "/")
elf_path = os.path.join(env.subst("$BUILD_DIR"), "firmware.elf")
openocd = os.path.join(env.subst("$PROJECT_PACKAGES_DIR"), "tool-openocd", "bin", "openocd.exe")
scripts_dir = os.path.join(env.subst("$PROJECT_PACKAGES_DIR"), "tool-openocd", "openocd", "scripts")

interface_cfg = "interface/stlink.cfg"
target_cfg = "target/stm32l4x.cfg"

# Ensure .bin is created before upload
def generate_bin(source, target, env):
    env.Execute(f"$OBJCOPY -O binary {elf_path} {firmware_path}")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", generate_bin)

# Custom upload process
def custom_upload(source, target, env):
    cmd1 = [
        openocd,
        "-s", scripts_dir,
        "-f", interface_cfg,
        "-f", target_cfg,
        "-c", f'program "{bootloader_path}" 0x8000000 verify reset exit'
    ]

    cmd2 = [
        openocd,
        "-s", scripts_dir,
        "-f", interface_cfg,
        "-f", target_cfg,
        "-c", f'program "{firmware_path}" 0x800A000 verify reset exit'
    ]

    try:
        print("Flashing bootloader")
        subprocess.run(cmd1, check=True)
        print("Flashing application firmware")
        subprocess.run(cmd2, check=True)
    except subprocess.CalledProcessError as e:
        print(f"\n\n--- Upload failed ---\nException: {e}\n---------------------\n")
        env.Exit(1)

env.Replace(UPLOADCMD=custom_upload)
