const robotTimeEl = document.getElementById("robotTime");
const topicBodyEl = document.getElementById("topicTableBody");
const imgEl = document.getElementById("cameraImage");
const tipEl = document.getElementById("imageTip");
const refreshBtn = document.getElementById("refreshImageBtn");

const escapeHtml = (text) =>
  String(text)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#39;");

function rowHtml(topic) {
  const stateClass = topic.online ? "state-online" : "state-offline";
  const stateText = topic.online ? "在线" : "离线";
  return `
    <tr>
      <td><span class="state-pill ${stateClass}">${stateText}</span></td>
      <td class="mono">${escapeHtml(topic.topic || "-")}</td>
      <td class="mono">${escapeHtml(topic.type || "-")}</td>
      <td>${Number(topic.hz || 0).toFixed(2)}</td>
      <td>${topic.count || 0}</td>
      <td class="mono">${escapeHtml(topic.last_recv_str || "-")}</td>
      <td class="preview">${escapeHtml(topic.preview || "-")}</td>
    </tr>
  `;
}

async function loadStatus() {
  try {
    const resp = await fetch("/api/status", { cache: "no-store" });
    if (!resp.ok) {
      throw new Error("status bad");
    }
    const data = await resp.json();
    robotTimeEl.textContent = data.robot_time || "--";
    topicBodyEl.innerHTML = (data.topics || []).map(rowHtml).join("");
  } catch (err) {
    robotTimeEl.textContent = "--";
    topicBodyEl.innerHTML =
      '<tr><td colspan="7" class="preview">状态获取失败，请检查后端和 ROS2 节点是否运行。</td></tr>';
  }
}

async function refreshImage() {
  refreshBtn.disabled = true;
  tipEl.textContent = "正在请求最新图像...";
  try {
    const ts = Date.now();
    const resp = await fetch(`/api/image?t=${ts}`, { cache: "no-store" });
    if (!resp.ok) {
      const err = await resp.json();
      throw new Error(err.message || "图像获取失败");
    }
    const blob = await resp.blob();
    const objectUrl = URL.createObjectURL(blob);
    imgEl.src = objectUrl;
    imgEl.style.display = "block";
    tipEl.textContent = `图像更新时间: ${new Date().toLocaleTimeString()}`;
  } catch (err) {
    tipEl.textContent = err.message || "图像获取失败";
  } finally {
    refreshBtn.disabled = false;
  }
}

refreshBtn.addEventListener("click", refreshImage);
loadStatus();
setInterval(loadStatus, 3000);
