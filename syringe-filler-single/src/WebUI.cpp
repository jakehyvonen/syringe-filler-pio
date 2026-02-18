/**
 * @file WebUI.cpp
 * @brief Embedded HTTP server for base syringe metadata.
 */
#include "WebUI.hpp"

#include <ArduinoJson.h>
#include <WebServer.h>

#include "Storage.hpp"

namespace {

WebServer server(80);
uint32_t g_currentRfid = 0;
WebUI::FillSpeedGetter g_fillSpeedGetter = nullptr;
WebUI::FillSpeedSetter g_fillSpeedSetter = nullptr;

constexpr size_t kMaxBaseList = 64;

const char kIndexHtml[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Syringe Filler Control</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 24px; background: #f7f7f7; }
    h1 { margin: 0 0 12px; }
    .panel { background: white; padding: 16px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); }
    .row { display: flex; gap: 16px; flex-wrap: wrap; }
    .col { flex: 1 1 280px; }
    ul { list-style: none; padding: 0; margin: 0; }
    li { padding: 6px 8px; border-bottom: 1px solid #eee; cursor: pointer; }
    li:hover { background: #f0f0f0; }
    label { display: block; margin-top: 8px; font-weight: 600; }
    input[type="text"], textarea { width: 100%; padding: 6px; box-sizing: border-box; }
    textarea { min-height: 90px; resize: vertical; }
    button { margin: 6px 6px 0 0; padding: 6px 10px; }
    .muted { color: #666; font-size: 0.9em; }
    .badge { padding: 2px 8px; background: #eef; border-radius: 12px; font-size: 0.9em; }
  </style>
</head>
<body>
  <h1>Single Syringe Filler Control</h1>
  <div class="row">
    <div class="col panel">
      <h3>Fill Speed</h3>
      <div class="muted">Adjust shared fill speed for manual syringe movement.</div>
      <label>Speed (steps/second)</label>
      <input id="fillSpeed" type="text" placeholder="e.g. 800" />
      <button id="loadSpeed">Load Speed</button>
      <button id="saveSpeed">Apply Speed</button>
    </div>
    <div class="col panel">
      <h3>Known Bases</h3>
      <button id="refresh">Refresh</button>
      <ul id="baseList"></ul>
    </div>
    <div class="col panel">
      <h3>Editor</h3>
      <div class="muted">Current tag: <span id="currentTag" class="badge">--</span></div>
      <button id="useCurrent">Use Current Tag</button>
      <label>RFID (hex)</label>
      <input id="rfid" type="text" placeholder="e.g. 1A2B3C4D" />
      <label>Paint Color</label>
      <input id="paintName" type="text" placeholder="e.g. Crimson Red" />
      <label>Recipe Name</label>
      <input id="recipeName" type="text" placeholder="e.g. Warm Sunset Mix" />
      <label>Recipe ID</label>
      <input id="recipeId" type="text" placeholder="e.g. 2024-05-A" />
      <label>Notes</label>
      <textarea id="notes" placeholder="Any extra metadata..."></textarea>
      <div id="status" class="muted"></div>
      <button id="save">Save</button>
      <button id="del">Delete</button>
    </div>
  </div>
  <script>
    const baseListEl = document.getElementById('baseList');
    const rfidEl = document.getElementById('rfid');
    const paintEl = document.getElementById('paintName');
    const recipeNameEl = document.getElementById('recipeName');
    const recipeIdEl = document.getElementById('recipeId');
    const notesEl = document.getElementById('notes');
    const statusEl = document.getElementById('status');
    const currentTagEl = document.getElementById('currentTag');
    const fillSpeedEl = document.getElementById('fillSpeed');

    function setStatus(msg, ok = true) {
      statusEl.textContent = msg;
      statusEl.style.color = ok ? '#2b6' : '#c33';
    }

    function clearForm() {
      paintEl.value = '';
      recipeNameEl.value = '';
      recipeIdEl.value = '';
      notesEl.value = '';
    }

    async function refreshList() {
      const resp = await fetch('/api/bases');
      const data = await resp.json();
      baseListEl.innerHTML = '';
      (data.bases || []).forEach(tag => {
        const li = document.createElement('li');
        li.textContent = tag;
        li.onclick = () => loadBase(tag);
        baseListEl.appendChild(li);
      });
      setStatus('Loaded base list.');
    }

    async function loadBase(tag) {
      rfidEl.value = tag;
      const resp = await fetch(`/api/bases/${tag}`);
      if (!resp.ok) {
        clearForm();
        setStatus('Base not found.', false);
        return;
      }
      const data = await resp.json();
      paintEl.value = data.paint_name || '';
      recipeNameEl.value = data.recipe_name || '';
      recipeIdEl.value = data.recipe_id || '';
      notesEl.value = data.notes || '';
      setStatus('Loaded base metadata.');
    }

    async function saveBase() {
      const rfid = rfidEl.value.trim();
      if (!rfid) return setStatus('RFID is required.', false);
      const body = {
        paint_name: paintEl.value.trim(),
        recipe_name: recipeNameEl.value.trim(),
        recipe_id: recipeIdEl.value.trim(),
        notes: notesEl.value.trim()
      };
      const resp = await fetch(`/api/bases/${rfid}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body)
      });
      if (resp.ok) {
        setStatus('Saved base metadata.');
        refreshList();
      } else {
        const msg = await resp.text();
        setStatus(`Save failed: ${msg}`, false);
      }
    }

    async function deleteBase() {
      const rfid = rfidEl.value.trim();
      if (!rfid) return setStatus('RFID is required.', false);
      if (!confirm('Delete this base?')) return;
      const resp = await fetch(`/api/bases/${rfid}`, { method: 'DELETE' });
      if (resp.ok) {
        setStatus('Deleted base.');
        clearForm();
        refreshList();
      } else {
        const msg = await resp.text();
        setStatus(`Delete failed: ${msg}`, false);
      }
    }


    async function loadFillSpeed() {
      const resp = await fetch('/api/fill-speed');
      if (!resp.ok) {
        setStatus('Failed to load fill speed.', false);
        return;
      }
      const data = await resp.json();
      fillSpeedEl.value = data.speed_sps ?? '';
      setStatus(`Loaded fill speed: ${data.speed_sps} sps.`);
    }

    async function saveFillSpeed() {
      const speed = parseInt(fillSpeedEl.value.trim(), 10);
      if (!Number.isFinite(speed) || speed <= 0) {
        setStatus('Fill speed must be a positive integer.', false);
        return;
      }
      const resp = await fetch('/api/fill-speed', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ speed_sps: speed })
      });
      if (!resp.ok) {
        const msg = await resp.text();
        setStatus(`Failed to set fill speed: ${msg}`, false);
        return;
      }
      const data = await resp.json();
      fillSpeedEl.value = data.speed_sps ?? speed;
      setStatus(`Applied fill speed: ${data.speed_sps} sps.`);
    }

    async function refreshCurrentTag() {
      const resp = await fetch('/api/rfid');
      if (!resp.ok) return;
      const data = await resp.json();
      currentTagEl.textContent = data.rfid || '--';
    }

    document.getElementById('refresh').onclick = refreshList;
    document.getElementById('save').onclick = saveBase;
    document.getElementById('del').onclick = deleteBase;
    document.getElementById('loadSpeed').onclick = loadFillSpeed;
    document.getElementById('saveSpeed').onclick = saveFillSpeed;
    document.getElementById('useCurrent').onclick = () => {
      const tag = currentTagEl.textContent;
      if (tag && tag !== '--') {
        rfidEl.value = tag;
        loadBase(tag);
      }
    };

    refreshList();
    refreshCurrentTag();
    loadFillSpeed();
    setInterval(refreshCurrentTag, 2000);
  </script>
</body>
</html>
)HTML";

String toHex(uint32_t rfid) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", rfid);
  return String(buf);
}

bool parseHex(const String& hexStr, uint32_t& out) {
  if (hexStr.length() == 0) return false;
  out = strtoul(hexStr.c_str(), nullptr, 16);
  return out != 0;
}

void sendJson(const JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  server.send(200, "application/json", body);
}

void handleListBases() {
  uint32_t ids[kMaxBaseList];
  size_t count = 0;
  if (!Storage::listBaseIds(ids, kMaxBaseList, count)) {
    server.send(500, "text/plain", "Failed to list bases");
    return;
  }

  JsonDocument doc;
  JsonArray arr = doc["bases"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    arr.add(toHex(ids[i]));
  }
  sendJson(doc);
}

void handleGetBase(uint32_t rfid) {
  Storage::BaseInfo info;
  if (!Storage::loadBase(rfid, info)) {
    server.send(404, "text/plain", "Base not found");
    return;
  }

  JsonDocument doc;
  doc["rfid"] = toHex(rfid);
  doc["paint_name"] = info.paintName;
  doc["recipe_name"] = info.recipeName;
  doc["recipe_id"] = info.recipeId;
  doc["notes"] = info.notes;
  sendJson(doc);
}

void handlePutBase(uint32_t rfid) {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  Storage::BaseInfo info;
  strlcpy(info.paintName, doc["paint_name"] | "", sizeof(info.paintName));
  strlcpy(info.recipeName, doc["recipe_name"] | "", sizeof(info.recipeName));
  strlcpy(info.recipeId, doc["recipe_id"] | "", sizeof(info.recipeId));
  strlcpy(info.notes, doc["notes"] | "", sizeof(info.notes));
  if (!Storage::saveBase(rfid, info)) {
    server.send(500, "text/plain", "Save failed");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleDeleteBase(uint32_t rfid) {
  if (!Storage::deleteBase(rfid)) {
    server.send(404, "text/plain", "Delete failed");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleApiBases() {
  if (server.method() == HTTP_GET) {
    handleListBases();
    return;
  }
  server.send(405, "text/plain", "Method not allowed");
}

void handleApiBaseItem() {
  String uri = server.uri();
  const String prefix = "/api/bases/";
  if (!uri.startsWith(prefix)) {
    server.send(404, "text/plain", "Not found");
    return;
  }

  String hexStr = uri.substring(prefix.length());
  uint32_t rfid = 0;
  if (!parseHex(hexStr, rfid)) {
    server.send(400, "text/plain", "Invalid RFID");
    return;
  }

  if (server.method() == HTTP_GET) {
    handleGetBase(rfid);
  } else if (server.method() == HTTP_PUT) {
    handlePutBase(rfid);
  } else if (server.method() == HTTP_DELETE) {
    handleDeleteBase(rfid);
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

void handleRfid() {
  JsonDocument doc;
  if (g_currentRfid != 0) {
    doc["rfid"] = toHex(g_currentRfid);
  } else {
    doc["rfid"] = "";
  }
  sendJson(doc);
}

void handleFillSpeedGet() {
  if (!g_fillSpeedGetter) {
    server.send(503, "text/plain", "Fill speed control unavailable");
    return;
  }
  JsonDocument doc;
  doc["speed_sps"] = g_fillSpeedGetter();
  sendJson(doc);
}

void handleFillSpeedPut() {
  if (!g_fillSpeedSetter) {
    server.send(503, "text/plain", "Fill speed control unavailable");
    return;
  }
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }

  JsonDocument req;
  DeserializationError err = deserializeJson(req, server.arg("plain"));
  if (err || req["speed_sps"].isNull()) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  uint32_t requested = req["speed_sps"].as<uint32_t>();
  uint32_t applied = g_fillSpeedSetter(requested);

  JsonDocument doc;
  doc["speed_sps"] = applied;
  sendJson(doc);
}

void handleFillSpeed() {
  if (server.method() == HTTP_GET) {
    handleFillSpeedGet();
    return;
  }
  if (server.method() == HTTP_PUT) {
    handleFillSpeedPut();
    return;
  }
  server.send(405, "text/plain", "Method not allowed");
}

}  // namespace

namespace WebUI {

void begin() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", kIndexHtml);
  });
  server.on("/index.html", HTTP_GET, []() {
    server.send_P(200, "text/html", kIndexHtml);
  });
  server.on("/api/bases", HTTP_ANY, handleApiBases);
  server.on("/api/rfid", HTTP_GET, handleRfid);
  server.on("/api/fill-speed", HTTP_ANY, handleFillSpeed);
  server.onNotFound(handleApiBaseItem);

  server.begin();
  Serial.println("[WebUI] HTTP server started on port 80.");
}

void handle() {
  server.handleClient();
}

void setCurrentRfid(uint32_t rfid) {
  g_currentRfid = rfid;
}

void setFillSpeedHooks(FillSpeedGetter getter, FillSpeedSetter setter) {
  g_fillSpeedGetter = getter;
  g_fillSpeedSetter = setter;
}

}  // namespace WebUI
