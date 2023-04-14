/*
 *  ros_camera_server - Intelligent camera stream server.
 *  Copyright (C) 2026  Stefan Fabian
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Affero General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Affero General Public License for more details.
 *
 *  You should have received a copy of the GNU Affero General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

// Embedded HTML/JS test page for WebRTC streaming.
// Served by the signaling server at the HTTP root (/).
//

#ifndef ROS_CAMERA_SERVER_WEBRTC_TEST_PAGE_HPP
#define ROS_CAMERA_SERVER_WEBRTC_TEST_PAGE_HPP

namespace ros_camera_server
{

// clang-format off
inline constexpr const char *WEBRTC_TEST_PAGE = R"HTML(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>ROS Camera Server - WebRTC Viewer</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: system-ui, sans-serif; background: #1a1a2e; color: #eee; padding: 20px; }
  h1 { font-size: 1.3em; margin-bottom: 15px; color: #a8d8ea; }
  .controls { display: flex; gap: 10px; margin-bottom: 15px; align-items: center; }
  input { padding: 8px 12px; border: 1px solid #444; border-radius: 4px; background: #16213e;
          color: #eee; font-size: 14px; width: 300px; }
  button { padding: 8px 16px; border: none; border-radius: 4px; cursor: pointer;
           font-size: 14px; font-weight: 500; }
  #connectBtn { background: #0f3460; color: #eee; }
  #connectBtn:hover { background: #1a4a7a; }
  #connectBtn.connected { background: #b33; }
  #connectBtn.connected:hover { background: #d44; }
  video { background: #000; border-radius: 4px; max-width: 100%; display: block; margin-top: 20px; }
  #status { font-size: 13px; color: #888; margin-bottom: 10px; }
  #status.connected { color: #6a6; }
  #status.error { color: #c66; }
  .info { font-size: 12px; color: #666; margin-top: 10px; }

  /* Camera List Styles */
  #camera-list { margin-bottom: 20px; display: grid; grid-template-columns: repeat(auto-fill, minmax(300px, 1fr)); gap: 10px; }
  .camera-item { background: #16213e; padding: 10px; border-radius: 6px; border: 1px solid #333; display: flex; justify-content: space-between; align-items: center; }
  .camera-info { font-size: 13px; color: #ccc; }
  .camera-name { font-weight: bold; color: #a8d8ea; margin-bottom: 2px; }
  .camera-meta { font-size: 11px; color: #888; }
  .camera-btn { background: #2a3a5a; color: #fff; padding: 6px 12px; font-size: 12px; }
  .camera-btn:hover { background: #3a4a6a; }
</style>
</head>
<body>
<h1>ROS Camera Server - WebRTC Viewer</h1>

<div id="camera-list">
  <!-- Populated by JS -->
  <div style="color: #666; font-style: italic;">Loading cameras...</div>
</div>

<div class="controls">
  <input id="pathInput" type="text" placeholder="WebSocket path, e.g. /camera_id/0">
  <button id="connectBtn" onclick="toggleConnection()">Connect</button>
</div>
<div id="status">Disconnected</div>
<video id="video" autoplay playsinline muted></video>
<div class="info" id="info"></div>

<script>
let ws = null;
let pc = null;

// On Load
window.addEventListener('DOMContentLoaded', fetchCameras);

async function fetchCameras() {
  try {
    const res = await fetch('/api/cameras');
    if (!res.ok) throw new Error(res.statusText);
    const cameras = await res.json();
    renderCameras(cameras);
  } catch (e) {
    console.error('Failed to fetch cameras:', e);
    document.getElementById('camera-list').innerHTML = '<div style="color: #c66;">Failed to load camera list</div>';
  }
}

function renderCameras(cameras) {
  const list = document.getElementById('camera-list');
  list.innerHTML = '';
  if (cameras.length === 0) {
    list.innerHTML = '<div style="color: #888;">No cameras found</div>';
    return;
  }

  cameras.forEach(cam => {
      const div = document.createElement('div');
      div.className = 'camera-item';

      const width = cam.width || '?';
      const height = cam.height || '?';
      const fps = cam.framerate || '?';
      const codec = cam.codec || 'unknown';

      div.innerHTML = `
        <div class="camera-info">
            <div class="camera-name">${cam.name}</div>
            <div class="camera-meta">${width}x${height} @ ${fps}fps (${codec})</div>
        </div>
        <button class="camera-btn" onclick="selectCamera('${cam.path}')">Select</button>
      `;
      list.appendChild(div);
  });
}

function selectCamera(path) {
  document.getElementById('pathInput').value = path;
  connect();
}

function setStatus(text, cls) {
  const el = document.getElementById('status');
  el.textContent = text;
  el.className = cls || '';
}

function toggleConnection() {
  if (ws) { disconnect(); }
  else { connect(); }
}

function connect() {
  const path = document.getElementById('pathInput').value.trim();
  if (!path) { setStatus('Please enter a path', 'error'); return; }

  // If already connected to a different path, disconnect first
  if (ws) disconnect();

  const wsUrl = 'ws://' + location.host + path;
  setStatus('Connecting to ' + wsUrl + '...');

  ws = new WebSocket(wsUrl);
  ws.onopen = () => {
    setStatus('WebSocket connected, waiting for offer...', 'connected');
    document.getElementById('connectBtn').textContent = 'Disconnect';
    document.getElementById('connectBtn').className = 'connected';
  };
  ws.onmessage = (evt) => handleMessage(JSON.parse(evt.data));
  ws.onerror = () => { setStatus('WebSocket error', 'error'); };
  ws.onclose = () => { disconnect(); };
}

function disconnect() {
  if (pc) { pc.close(); pc = null; }
  if (ws) { ws.close(); ws = null; }
  document.getElementById('video').srcObject = null;
  document.getElementById('connectBtn').textContent = 'Connect';
  document.getElementById('connectBtn').className = '';
  document.getElementById('info').textContent = '';
  setStatus('Disconnected');
}

function handleMessage(msg) {
  if (msg.type === 'offer') {
    handleOffer(msg);
  } else if (msg.ice) {
    if (pc) { pc.addIceCandidate(new RTCIceCandidate(msg.ice)); }
  }
}

async function handleOffer(msg) {
  setStatus('Received offer, creating answer...', 'connected');
  pc = new RTCPeerConnection({ iceServers: [] });

  pc.ontrack = (evt) => {
    document.getElementById('video').srcObject = evt.streams[0];
    setStatus('Receiving video', 'connected');
  };

  pc.onicecandidate = (evt) => {
    if (evt.candidate && ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ ice: evt.candidate.toJSON() }));
    }
  };

  pc.oniceconnectionstatechange = () => {
    const state = pc.iceConnectionState;
    if (state === 'connected' || state === 'completed') {
      setStatus('Connected - streaming', 'connected');
    } else if (state === 'disconnected' || state === 'failed' || state === 'closed') {
      setStatus('Peer connection ' + state, 'error');
    }
    document.getElementById('info').textContent = 'ICE: ' + state;
  };

  // Add transceiver to receive video
  pc.addTransceiver('video', { direction: 'recvonly' });

  await pc.setRemoteDescription(new RTCSessionDescription({ type: 'offer', sdp: msg.sdp }));
  const answer = await pc.createAnswer();
  await pc.setLocalDescription(answer);

  ws.send(JSON.stringify({ type: 'answer', sdp: answer.sdp }));
}
</script>
</body>
</html>
)HTML";
// clang-format on

} // namespace ros_camera_server

#endif // ROS_CAMERA_SERVER_WEBRTC_TEST_PAGE_HPP
