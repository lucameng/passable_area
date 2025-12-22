#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEB_DIR="${ROOT_DIR}/debian"
OVR_DIR="${ROOT_DIR}/packaging/debian"
PKG="ros-foxy-passable-area"

# 0) 确保 bloom 已生成 debian/
test -d "${DEB_DIR}" || { echo "debian/ not found. Run bloom-generate first."; exit 1; }

# 1) 注入 unit：让 dh_installsystemd 识别并安装
# 文件名需带包名前缀和 unit 名，匹配 --name passable_area
install -m 0644 "${ROOT_DIR}/packaging/passable_area.service" "${DEB_DIR}/${PKG}.passable_area.service"

# 2) 注入维护脚本
install -m 0755 "${OVR_DIR}/${PKG}.postinst" "${DEB_DIR}/${PKG}.postinst"

# 3) 注入 prerm
if [ -f "${OVR_DIR}/${PKG}.prerm" ]; then
  install -m 0755 "${OVR_DIR}/${PKG}.prerm" "${DEB_DIR}/${PKG}.prerm"
fi

# 3.1) 注入 postrm
if [ -f "${OVR_DIR}/${PKG}.postrm" ]; then
  install -m 0755 "${OVR_DIR}/${PKG}.postrm" "${DEB_DIR}/${PKG}.postrm"
fi

# 4) 追加 rules override
if ! grep -q '^override_dh_installsystemd:' "${DEB_DIR}/rules"; then
  printf "\n" >> "${DEB_DIR}/rules"
  cat "${OVR_DIR}/rules.append" >> "${DEB_DIR}/rules"
fi

# 5) 确保 dh 调用包含 --with systemd（老版本 debhelper 默认不带）
if ! grep -q -- '--with systemd' "${DEB_DIR}/rules"; then
  sed -i 's/^\(\s*dh\s\+\$@\)/\1 --with systemd/' "${DEB_DIR}/rules"
fi

# 6) 确保 .install 包含 service（双保险，即便 dh_installsystemd 未生效也能打包）
INSTALL_FILE="${DEB_DIR}/${PKG}.install"
if [ ! -f "${INSTALL_FILE}" ]; then
  touch "${INSTALL_FILE}"
fi
if ! grep -q '^packaging/passable_area.service' "${INSTALL_FILE}"; then
  echo "packaging/passable_area.service lib/systemd/system/" >> "${INSTALL_FILE}"
fi
