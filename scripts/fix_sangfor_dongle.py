"""
深信服 Elite4 精锐5 加密锁诊断修复脚本
在工控机上运行: python fix_sangfor_dongle.py
"""

import subprocess
import os
import sys

def run(cmd):
    return subprocess.run(cmd, shell=True, capture_output=True, text=True)


print("=" * 60)
print("深信服 Elite4 加密锁诊断")
print("=" * 60)

# 1. 找到狗的 USB 设备
print("\n[1] USB 设备识别...")
r = run('wmic path Win32_PnPEntity where "Name like \'%Elite%\' or Name like \'%精锐%\' or Name like \'%Sense%\' or Name like \'%Sangfor%\'" get Name,DeviceID,Status')
print(r.stdout or "  未找到 Elite 设备")
if r.stderr:
    print(f"  ERR: {r.stderr}")

# 2. 枚举所有 HID/USB 找可疑 VID
print("\n[2] 所有 USB 设备...")
r = run('wmic path Win32_PnPEntity where "Name like \'%USB%\' or Name like \'%HID%\'" get Name,DeviceID | findstr "VID_"')
for line in (r.stdout or "").split("\n"):
    line = line.strip()
    if line and "VID_" in line:
        # 标记已知加密狗 VID
        tags = []
        for vid, name in [("0529", "HASP/Sentinel"), ("096E", "Rockey"),
                           ("3386", "SenseLock/Elite"), ("064F", "CodeMeter"),
                           ("3689", "Feitian"), ("2B9F", "DeepSea")]:
            if f"VID_{vid}" in line:
                tags.append(name)
        tag = f"  <-- 可能是 {'/'.join(tags)}" if tags else ""
        print(f"  {line}{tag}")

# 3. 找深信服驱动文件
print("\n[3] 搜索深信服驱动文件...")
for root in [r"C:\Windows\System32\drivers", r"C:\Windows\SysWOW64\drivers",
             r"C:\Program Files", r"C:\Program Files (x86)"]:
    if not os.path.exists(root):
        continue
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if f.lower().endswith((".sys", ".dll")) and any(
                kw in f.lower() for kw in ["sang", "elite", "sense", "精锐",
                                            "snslock", "usbkey", "dog"]):
                full = os.path.join(dirpath, f)
                print(f"  找到: {full}")

# 4. 验签
print("\n[4] 驱动签名验证...")
r = run('driverquery /v | findstr -i "elite sang sense"')
if r.stdout.strip():
    for line in r.stdout.split("\n"):
        print(f"  {line.strip()}")
else:
    print("  未找到运行中的深信服驱动")

# 5. 服务状态
print("\n[5] 深信服服务...")
r = run('sc query type= service state= all | findstr -i "sang elite sense lock auth"')
for line in (r.stdout or "").split("\n"):
    line = line.strip()
    if line:
        print(f"  {line}")
if not r.stdout or not r.stdout.strip():
    print("  未找到深信服服务")

# 6. 建议
print("\n" + "=" * 60)
print("修复建议:")
print("=" * 60)
print("""
如果 Step3 找不到 .sys 文件:
  → 重装深信服云软锁 SDK（从深信服官网或软件供应商获取）

如果 Step4 驱动未运行:
  → 管理员 PowerShell: sc start <驱动服务名>

如果 Step4 驱动有但签名无效:
  → bcdedit /set testsigning on  (需要管理员，需重启)
  → 或联系深信服获取支持 Secure Boot 的新版驱动

如果狗灯不亮:
  → 换主板后面 USB 口，不要用 HUB/延长线
  → 换一台机器测试，确认狗硬件是否正常
""")
