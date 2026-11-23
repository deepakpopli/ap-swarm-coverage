// Live drone cameras: WebRTC from MediaMTX (tools/mediamtx.yml) through its WHEP endpoint.
// The drones publish rtsp://127.0.0.1:8554/droneN; the page plays http://<its host>:8889/droneN/whep.
"use strict";
(() => {
  const N = 3, RETRY_MS = 3000, WHEP_PORT = 8889;
  const COLORS = ["#1f5fd6", "#0b6b2a", "#111111"];   // until /api/runs gives geometry.colors
  const box = document.getElementById("videobox");
  if (!box) return;
  const players = [];

  function whepUrl(id) {
    const host = location.hostname.includes(":") ? `[${location.hostname}]` : location.hostname;
    return `${location.protocol}//${host}:${WHEP_PORT}/drone${id}/whep`;
  }

  function waitIce(pc) {             // MediaMTX takes the offer with all candidates in it
    if (pc.iceGatheringState === "complete") return Promise.resolve();
    return new Promise((done) => {
      const t = setTimeout(done, 2000);
      pc.addEventListener("icegatheringstatechange", () => {
        if (pc.iceGatheringState === "complete") { clearTimeout(t); done(); }
      });
    });
  }

  function makePlayer(id) {
    const cell = document.createElement("div");
    cell.style.cssText = "min-width:0;position:relative";
    cell.innerHTML = `<div class="vlabel" style="font-size:11px;color:#fff;padding:1px 6px;border-radius:3px 3px 0 0;display:inline-block">drone ${id}</div>` +
      `<video muted autoplay playsinline style="width:100%;aspect-ratio:16/9;background:#000;display:block;border-radius:0 3px 3px 3px"></video>` +
      `<div class="vmsg dim" style="position:absolute;left:0;right:0;top:45%;text-align:center">no video</div>`;
    const p = {id, cell, video: cell.querySelector("video"), msg: cell.querySelector(".vmsg"),
      label: cell.querySelector(".vlabel"), pc: null, timer: null, on: false};
    p.label.style.background = COLORS[(id - 1) % COLORS.length];
    p.video.onplaying = () => (p.msg.hidden = true);
    p.video.title = "click: full screen";
    p.video.style.cursor = "zoom-in";
    p.video.onclick = () => (document.fullscreenElement ? document.exitFullscreen()
      : p.video.requestFullscreen && p.video.requestFullscreen());
    return p;
  }

  function stop(p, retry) {
    clearTimeout(p.timer);
    if (p.pc) { p.pc.close(); p.pc = null; }
    p.video.srcObject = null; p.msg.hidden = false; p.msg.textContent = p.on ? "no video" : "live only";
    if (retry && p.on) p.timer = setTimeout(() => connect(p), RETRY_MS);
  }

  async function connect(p) {
    stop(p, false);
    const pc = new RTCPeerConnection();   // no ICE servers: MediaMTX gives its LAN / VPN addresses
    p.pc = pc;
    pc.addTransceiver("video", {direction: "recvonly"});
    pc.ontrack = (e) => { p.video.srcObject = e.streams[0] || new MediaStream([e.track]); };
    pc.onconnectionstatechange = () => {
      if (p.pc === pc && ["failed", "disconnected", "closed"].includes(pc.connectionState)) stop(p, true);
    };
    try {
      await pc.setLocalDescription(await pc.createOffer());
      await waitIce(pc);
      const r = await fetch(whepUrl(p.id), {method: "POST", headers: {"Content-Type": "application/sdp"},
        body: pc.localDescription.sdp});
      if (r.status !== 201) throw new Error("WHEP " + r.status);   // 404: the drone is not publishing
      if (p.pc !== pc) return;
      await pc.setRemoteDescription({type: "answer", sdp: await r.text()});
    } catch (err) {
      if (p.pc === pc) stop(p, true);
    }
  }

  function setLive(on) {             // app.js keeps `mode`: "live" or "replay"
    for (const p of players) {
      if (p.on === on) continue;
      p.on = on;
      if (on) connect(p); else stop(p, false);
    }
  }

  const row = document.createElement("div");   // two on top, the third centred below
  row.style.cssText = "display:grid;grid-template-columns:1fr 1fr;gap:8px";
  for (let id = 1; id <= N; id++) { const p = makePlayer(id); players.push(p); row.appendChild(p.cell); }
  if (N % 2) players[N - 1].cell.style.cssText += ";grid-column:1 / span 2;justify-self:center;width:calc(50% - 4px)";
  box.appendChild(row);
  players.forEach((p) => stop(p, false));
  setInterval(() => {
    if (typeof geo !== "undefined" && geo && geo.colors)
      players.forEach((p) => (p.label.style.background = geo.colors[(p.id - 1) % geo.colors.length]));
    setLive(typeof mode !== "undefined" && mode === "live");
  }, 1000);
})();
