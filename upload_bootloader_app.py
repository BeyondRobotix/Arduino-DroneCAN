Import("env")
import os
import subprocess

# Project-relative paths
bootloader_path = os.path.abspath("MicroNodeBootloader.bin")
firmware_path = os.path.abspath(os.path.join(env.subst("$BUILD_DIR"), "firmware.bin"))

# Correct OpenOCD executable
openocd = os.path.join(env.subst("$PROJECT_PACKAGES_DIR"), "tool-openocd", "bin", "openocd.exe")

# Fix for OpenOCD scripts: use the scripts from board config
interface_cfg = "interface/stlink.cfg"
target_cfg = "target/stm32L4x.cfg"

# Full command
def custom_upload(source, target, env):
    cmd = [
        openocd,
        "-s", os.path.join(env.subst("$PROJECT_PACKAGES_DIR"), "tool-openocd", "scripts"),  # this tells OpenOCD where to find .cfg files
        "-f", interface_cfg,
        "-f", target_cfg,
        "-c", f"program {bootloader_path} 0x08000000 verify",
        "-c", f"program {firmware_path} 0x0800A000 verify reset exit"
    ]

    print("Flashing bootloader + application...")
    try:
        result = subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print("Upload failed.")
        env.Exit(1)

env.Replace(UPLOADCMD=custom_upload)
