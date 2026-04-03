/**
 * @file WebUI.cpp
 * @brief Embedded HTTP server for recipe CRUD and a tiny UI.
 */
#include "app/WebUI.hpp"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <stdarg.h>

#include <WifiCredentials.hpp>
#include <WifiManager.hpp>

#include "app/CommandRouter.hpp"
#include "hw/Pots.hpp"
#include "util/Recipe.hpp"
#include "util/Storage.hpp"

namespace {

WebServer server(80);

constexpr size_t kMaxRecipeList = 64;
constexpr size_t kSerialLogCapacity = 257;
constexpr size_t kSerialPollBatch = 89;
Shared::WifiManager g_wifi;

struct SerialLogEntry {
  uint32_t seq;
  String line;
};

SerialLogEntry g_serialLog[kSerialLogCapacity];
size_t g_serialLogStart = 0;
size_t g_serialLogCount = 0;
uint32_t g_serialLogNextSeq = 1;

void appendSerialLog(const String &line) {
  const size_t idx = (g_serialLogStart + g_serialLogCount) % kSerialLogCapacity;
  g_serialLog[idx].seq = g_serialLogNextSeq++;
  g_serialLog[idx].line = line;
  if (g_serialLogCount < kSerialLogCapacity) {
    ++g_serialLogCount;
  } else {
    g_serialLogStart = (g_serialLogStart + 1) % kSerialLogCapacity;
  }
}

void serialPrintfAndLog(const char *fmt, ...) {
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  Serial.print(buffer);
  String msg = String(buffer);
  msg.replace("\r", "");
  msg.replace("\n", "");
  if (msg.length() > 0) appendSerialLog(msg);
}

void serialPrintlnAndLog(const String &line) {
  Serial.println(line);
  appendSerialLog(line);
}

const char kIndexHtml[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Syringe Filler Recipes</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 24px; background: #f7f7f7; }
    h1 { margin: 0 0 12px; }
    .panel { background: white; padding: 16px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); }
    .row { display: flex; gap: 16px; flex-wrap: wrap; }
    .col { flex: 1 1 260px; }
    ul { list-style: none; padding: 0; margin: 0; }
    li { padding: 6px 8px; border-bottom: 1px solid #eee; cursor: pointer; }
    li:hover { background: #f0f0f0; }
    table { width: 100%; border-collapse: collapse; margin-top: 8px; }
    th, td { border-bottom: 1px solid #eee; padding: 6px; text-align: left; }
    input[type="number"], input[type="text"] { width: 100%; padding: 6px; }
    button { margin: 4px 4px 4px 0; padding: 6px 10px; }
    .muted { color: #666; font-size: 0.9em; }
    .chip { display: inline-block; border-radius: 999px; padding: 4px 10px; margin: 0 6px 6px 0; background: #ececec; color: #444; }
    .chip.ok { background: #d8f6e4; color: #186a3b; }
    .chip.bad { background: #ffe0e0; color: #9b2226; }
    .console { background: #111; color: #e8e8e8; font-family: monospace; padding: 12px; border-radius: 8px; min-height: 180px; max-height: 320px; overflow-y: auto; white-space: pre-wrap; }
    .consoleLine { margin: 0; }
  </style>
</head>
<body>
  <h1>Syringe Filler Recipes</h1>
  <div class="row">
    <div class="col panel">
      <h3>Recipes</h3>
      <button id="refresh">Refresh</button>
      <ul id="recipeList"></ul>
    </div>
    <div class="col panel">
      <h3>Editor</h3>
      <label>Recipe ID (hex)</label>
      <input id="recipeId" type="text" placeholder="e.g. 1A2B3C4D" />
      <div class="muted">Volume uses mL. Base slot is 0-based.</div>
      <table>
        <thead>
          <tr><th>Volume (mL)</th><th>Base Slot</th><th></th></tr>
        </thead>
        <tbody id="steps"></tbody>
      </table>
      <button id="add">Add Step</button>
      <button id="save">Save</button>
      <button id="del">Delete</button>
      <div id="status" class="muted"></div>
    </div>
  </div>

  <div class="row" style="margin-top: 16px;">
    <div class="col panel">
      <h3>Machine Controls</h3>
      <button id="cmdHome">home</button>
      <button id="cmdScanAll">scanall</button>
      <button id="cmdScanTool">scantool</button>
      <button id="cmdInitAll">initializeall</button>
      <button id="cmdLoad">load</button>
      <button id="cmdRun">run</button>
      <label>Run repeats</label>
      <input id="repeatCount" type="number" min="1" value="5" />
      <div class="muted">run repeats execute the currently loaded recipe count times.</div>
      <label>Raw command</label>
      <input id="rawCommand" type="text" placeholder="e.g. sfc.run 3" />
      <button id="cmdSendRaw">Send</button>
      <div id="commandStatus" class="muted"></div>
    </div>

    <div class="col panel">
      <h3>Init Status</h3>
      <div id="initIndicators" class="muted">Not initialized yet.</div>
      <h3>Latest VREF</h3>
      <div id="vrefStatus" class="muted">--</div>
    </div>
  </div>

  <div class="row" style="margin-top: 16px;">
    <div class="col panel">
      <h3>Serial Console</h3>
      <div class="muted">Shows recent serial-style output for commands executed via Web UI.</div>
      <div id="serialConsole" class="console"></div>
    </div>
  </div>

  <script>
    const listEl = document.getElementById('recipeList');
    const stepsEl = document.getElementById('steps');
    const recipeIdEl = document.getElementById('recipeId');
    const statusEl = document.getElementById('status');
    const commandStatusEl = document.getElementById('commandStatus');
    const initIndicatorsEl = document.getElementById('initIndicators');
    const vrefStatusEl = document.getElementById('vrefStatus');
    const rawCommandEl = document.getElementById('rawCommand');
    const serialConsoleEl = document.getElementById('serialConsole');
    let serialCursor = 0;

    function appendConsoleLine(line) {
      const row = document.createElement('div');
      row.className = 'consoleLine';
      row.textContent = line;
      serialConsoleEl.appendChild(row);
      while (serialConsoleEl.childNodes.length > 257) {
        serialConsoleEl.removeChild(serialConsoleEl.firstChild);
      }
      serialConsoleEl.scrollTop = serialConsoleEl.scrollHeight;
    }

    function setStatus(msg, ok = true) {
      statusEl.textContent = msg;
      statusEl.style.color = ok ? '#2b6' : '#c33';
    }

    function setCommandStatus(msg, ok = true) {
      commandStatusEl.textContent = msg;
      commandStatusEl.style.color = ok ? '#2b6' : '#c33';
    }

    function addRow(step = { volume_ml: 1, base_slot: 0 }) {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><input type="number" step="0.1" min="0.1" value="${step.volume_ml}" /></td>
        <td><input type="number" min="0" value="${step.base_slot}" /></td>
        <td><button class="remove">Remove</button></td>
      `;
      tr.querySelector('.remove').onclick = () => tr.remove();
      stepsEl.appendChild(tr);
    }

    function renderInitIndicators(result) {
      const steps = result?.data?.steps;
      if (!steps) {
        initIndicatorsEl.innerHTML = '<span class="chip">No init data yet</span>';
        return;
      }
      const order = ['home', 'scantool', 'sfc.scanall', 'recipe.list', 'sfc.status'];
      initIndicatorsEl.innerHTML = order.map(key => {
        const entry = steps[key] || { ok: false, message: 'missing' };
        const cls = entry.ok ? 'chip ok' : 'chip bad';
        return `<span class="${cls}">${key}: ${entry.ok ? 'ok' : 'error'} (${entry.message || ''})</span>`;
      }).join('');
    }

    async function sendCommand(command) {
      const resp = await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command })
      });
      if (!resp.ok) {
        const txt = await resp.text();
        throw new Error(txt || 'command failed');
      }
      return resp.json();
    }

    async function refreshVref() {
      try {
        const resp = await fetch('/api/telemetry');
        const data = await resp.json();
        const vref = Number(data.latest_vref_volts);
        if (Number.isFinite(vref)) {
          vrefStatusEl.textContent = `${vref.toFixed(3)} V`;
          vrefStatusEl.style.color = vref >= 3.0 ? '#2b6' : '#c33';
        } else {
          vrefStatusEl.textContent = '--';
        }
      } catch (err) {
        vrefStatusEl.textContent = `telemetry error: ${err.message}`;
        vrefStatusEl.style.color = '#c33';
      }
    }

    async function refreshSerialConsole() {
      try {
        const resp = await fetch(`/api/serial?since=${serialCursor}`);
        const data = await resp.json();
        (data.lines || []).forEach(entry => appendConsoleLine(entry.text));
        serialCursor = Number(data.next_seq || serialCursor);
      } catch (err) {
        appendConsoleLine(`[web] serial poll error: ${err.message}`);
      }
    }

    async function refreshList() {
      const resp = await fetch('/api/recipes');
      const data = await resp.json();
      listEl.innerHTML = '';
      (data.recipes || []).forEach(recipeId => {
        const li = document.createElement('li');
        li.textContent = recipeId;
        li.onclick = () => loadRecipe(recipeId);
        listEl.appendChild(li);
      });
      setStatus('Loaded recipes.');
    }

    async function loadRecipe(recipeId) {
      recipeIdEl.value = recipeId;
      const resp = await fetch(`/api/recipes/${recipeId}`);
      if (!resp.ok) {
        setStatus('Recipe not found.', false);
        stepsEl.innerHTML = '';
        return;
      }
      const data = await resp.json();
      stepsEl.innerHTML = '';
      (data.steps || []).forEach(addRow);
      if ((data.steps || []).length === 0) addRow();
      setStatus('Recipe loaded.');
    }

    async function saveRecipe() {
      const recipeId = recipeIdEl.value.trim();
      if (!recipeId) return setStatus('Recipe ID is required.', false);
      const steps = Array.from(stepsEl.querySelectorAll('tr')).map(row => {
        const inputs = row.querySelectorAll('input');
        return {
          volume_ml: parseFloat(inputs[0].value || '0'),
          base_slot: parseInt(inputs[1].value || '0', 10)
        };
      });
      const resp = await fetch(`/api/recipes/${recipeId}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ steps })
      });
      if (resp.ok) {
        setStatus('Recipe saved.');
        refreshList();
      } else {
        const msg = await resp.text();
        setStatus(`Save failed: ${msg}`, false);
      }
    }

    async function deleteRecipe() {
      const recipeId = recipeIdEl.value.trim();
      if (!recipeId) return setStatus('Recipe ID is required.', false);
      if (!confirm('Delete this recipe?')) return;
      const resp = await fetch(`/api/recipes/${recipeId}`, { method: 'DELETE' });
      if (resp.ok) {
        setStatus('Recipe deleted.');
        stepsEl.innerHTML = '';
        refreshList();
      } else {
        const msg = await resp.text();
        setStatus(`Delete failed: ${msg}`, false);
      }
    }

    document.getElementById('refresh').onclick = refreshList;
    document.getElementById('add').onclick = () => addRow();
    document.getElementById('save').onclick = saveRecipe;
    document.getElementById('del').onclick = deleteRecipe;

    document.getElementById('cmdHome').onclick = async () => {
      const r = await sendCommand('home');
      setCommandStatus(r.message || 'home complete', r.status === 'ok');
      refreshVref();
    };
    document.getElementById('cmdScanAll').onclick = async () => {
      const r = await sendCommand('sfc.scanall');
      setCommandStatus(r.message || 'scanall complete', r.status === 'ok');
      refreshVref();
    };
    document.getElementById('cmdScanTool').onclick = async () => {
      const r = await sendCommand('scantool');
      setCommandStatus(r.message || 'scantool complete', r.status === 'ok');
      refreshVref();
    };
    document.getElementById('cmdInitAll').onclick = async () => {
      const r = await sendCommand('initializeall');
      setCommandStatus(r.message || 'initializeall complete', r.status === 'ok');
      renderInitIndicators(r);
      refreshVref();
    };
    document.getElementById('cmdLoad').onclick = async () => {
      const recipeId = recipeIdEl.value.trim();
      if (!recipeId) return setCommandStatus('Recipe ID is required for load.', false);
      const r = await sendCommand(`sfc.load ${recipeId}`);
      setCommandStatus(r.message || 'load complete', r.status === 'ok');
      refreshVref();
    };
    document.getElementById('cmdRun').onclick = async () => {
      const repeats = parseInt(document.getElementById('repeatCount').value || '1', 10);
      if (!Number.isFinite(repeats) || repeats < 1) {
        setCommandStatus('Repeat count must be >= 1', false);
        return;
      }
      const r = await sendCommand(`sfc.run ${repeats}`);
      setCommandStatus(r.message || 'run complete', r.status === 'ok');
      refreshVref();
    };
    document.getElementById('cmdSendRaw').onclick = async () => {
      const raw = rawCommandEl.value.trim();
      if (!raw) {
        setCommandStatus('Raw command is required.', false);
        return;
      }
      const r = await sendCommand(raw);
      setCommandStatus(r.message || `${raw} complete`, r.status === 'ok');
      refreshVref();
    };
    rawCommandEl.addEventListener('keydown', async (evt) => {
      if (evt.key !== 'Enter') return;
      evt.preventDefault();
      document.getElementById('cmdSendRaw').click();
    });

    refreshList();
    refreshVref();
    refreshSerialConsole();
    setInterval(refreshVref, 2053);
    setInterval(refreshSerialConsole, 2053);
  </script>
</body>
</html>
)HTML";

String recipeIdToHex(uint32_t recipeId) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", recipeId);
  return String(buf);
}

bool parseRecipeId(const String& hexStr, uint32_t& out) {
  if (hexStr.length() == 0) return false;
  out = strtoul(hexStr.c_str(), nullptr, 16);
  return out != 0;
}

void sendJson(const JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  server.send(200, "application/json", body);
}

void handleCommandApi() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method not allowed");
    return;
  }
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }
  JsonDocument requestDoc;
  DeserializationError err = deserializeJson(requestDoc, server.arg("plain"));
  if (err) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  String command = requestDoc["command"].as<String>();
  command.trim();
  if (command.length() == 0) {
    server.send(400, "text/plain", "Missing command");
    return;
  }

  appendSerialLog(String("> ") + command);
  String responseJson;
  CommandRouter::executeCommandLine(command, responseJson);
  server.send(200, "application/json", responseJson);
}

void handleSerialApi() {
  uint32_t since = 0;
  if (server.hasArg("since")) {
    since = static_cast<uint32_t>(strtoul(server.arg("since").c_str(), nullptr, 10));
  }

  JsonDocument doc;
  JsonArray lines = doc["lines"].to<JsonArray>();
  size_t emitted = 0;
  for (size_t i = 0; i < g_serialLogCount && emitted < kSerialPollBatch; ++i) {
    const size_t idx = (g_serialLogStart + i) % kSerialLogCapacity;
    const SerialLogEntry &entry = g_serialLog[idx];
    if (entry.seq <= since) continue;
    JsonObject row = lines.add<JsonObject>();
    row["seq"] = entry.seq;
    row["text"] = entry.line;
    ++emitted;
  }
  doc["next_seq"] = g_serialLogNextSeq;
  sendJson(doc);
}

void handleTelemetryApi() {
  JsonDocument doc;
  doc["latest_vref_volts"] = Pots::latestVrefVolts();
  sendJson(doc);
}

void handleListRecipes() {
  serialPrintfAndLog("[WebUI] GET /api/recipes\n");
  uint32_t ids[kMaxRecipeList];
  size_t count = 0;
  if (!Util::listRecipeIds(ids, kMaxRecipeList, count)) {
    serialPrintlnAndLog("[WebUI] listRecipeIds() failed");
    server.send(500, "text/plain", "Failed to list recipes");
    return;
  }

  JsonDocument doc;
  JsonArray arr = doc["recipes"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    arr.add(recipeIdToHex(ids[i]));
  }
  sendJson(doc);
}

void handleGetRecipe(uint32_t recipeId) {
  serialPrintfAndLog("[WebUI] GET /api/recipes/%08X\n", recipeId);
  Util::Recipe recipe;
  if (!Util::loadRecipe(recipeId, recipe)) {
    serialPrintfAndLog("[WebUI] loadRecipe() failed for %08X\n", recipeId);
    server.send(404, "text/plain", "Recipe not found");
    return;
  }

  JsonDocument doc;
  doc["recipe_id"] = recipeIdToHex(recipeId);
  JsonArray arr = doc["steps"].to<JsonArray>();
  recipe.toJson(arr);
  sendJson(doc);
}

void handlePutRecipe(uint32_t recipeId) {
  serialPrintfAndLog("[WebUI] PUT /api/recipes/%08X\n", recipeId);
  if (!server.hasArg("plain")) {
    serialPrintlnAndLog("[WebUI] PUT missing body");
    server.send(400, "text/plain", "Missing body");
    return;
  }

  serialPrintfAndLog("[WebUI] PUT payload: %s\n", server.arg("plain").c_str());
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    serialPrintfAndLog("[WebUI] PUT JSON parse error: %s\n", err.c_str());
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  JsonArrayConst arr;
  if (doc.is<JsonArray>()) {
    arr = doc.as<JsonArrayConst>();
  } else {
    arr = doc["steps"].as<JsonArrayConst>();
  }

  if (arr.isNull()) {
    serialPrintlnAndLog("[WebUI] PUT missing steps array");
    server.send(400, "text/plain", "Missing steps array");
    return;
  }

  Util::Recipe recipe;
  recipe.fromJson(arr);
  if (recipe.isEmpty()) {
    serialPrintlnAndLog("[WebUI] PUT recipe invalid or empty after parsing");
    server.send(400, "text/plain", "Recipe invalid or empty");
    return;
  }

  serialPrintfAndLog("[WebUI] PUT parsed %u step(s)\n", recipe.count);
  for (uint8_t i = 0; i < recipe.count; ++i) {
    serialPrintfAndLog("[WebUI] step %u: volume_ml=%.3f base_slot=%d\n",
                       i, recipe.steps[i].volumeMl, recipe.steps[i].baseSlot);
  }
  if (!Util::saveRecipe(recipeId, recipe)) {
    serialPrintfAndLog("[WebUI] saveRecipe() failed for %08X\n", recipeId);
    server.send(500, "text/plain", "Save failed");
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleDeleteRecipe(uint32_t recipeId) {
  serialPrintfAndLog("[WebUI] DELETE /api/recipes/%08X\n", recipeId);
  if (!Util::deleteRecipe(recipeId)) {
    serialPrintfAndLog("[WebUI] deleteRecipe() failed for %08X\n", recipeId);
    server.send(404, "text/plain", "Delete failed");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleApiRecipes() {
  serialPrintfAndLog("[WebUI] %s /api/recipes\n", server.method() == HTTP_GET ? "GET" : "OTHER");
  if (server.method() == HTTP_GET) {
    handleListRecipes();
    return;
  }
  server.send(405, "text/plain", "Method not allowed");
}

void handleApiRecipeItem() {
  String uri = server.uri();
  const String prefix = "/api/recipes/";
  if (!uri.startsWith(prefix)) {
    serialPrintfAndLog("[WebUI] Not found: %s\n", uri.c_str());
    server.send(404, "text/plain", "Not found");
    return;
  }

  String hexStr = uri.substring(prefix.length());
  uint32_t recipeId = 0;
  if (!parseRecipeId(hexStr, recipeId)) {
    serialPrintfAndLog("[WebUI] Invalid recipe ID from URI: %s\n", uri.c_str());
    server.send(400, "text/plain", "Invalid recipe ID");
    return;
  }

  if (server.method() == HTTP_GET) {
    handleGetRecipe(recipeId);
  } else if (server.method() == HTTP_PUT) {
    handlePutRecipe(recipeId);
  } else if (server.method() == HTTP_DELETE) {
    handleDeleteRecipe(recipeId);
  } else {
    serialPrintfAndLog("[WebUI] Method not allowed for %s\n", uri.c_str());
    server.send(405, "text/plain", "Method not allowed");
  }
}

void startWiFi() {
  String ssid;
  String password;
  if (Shared::WiFiCredentials::load(ssid, password)) {
    if (g_wifi.connect(ssid, password)) {
      return;
    }
    serialPrintlnAndLog("[WebUI] Falling back to AP mode.");
  } else {
    serialPrintlnAndLog("[WebUI] No saved WiFi credentials found.");
  }
  g_wifi.startAccessPoint();
  serialPrintlnAndLog("[WebUI] Open http://192.168.4.1/ to configure.");
}

} // namespace

namespace App {
namespace WebUI {

void begin() {
  startWiFi();

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", kIndexHtml);
  });
  server.on("/index.html", HTTP_GET, []() {
    server.send_P(200, "text/html", kIndexHtml);
  });
  server.on("/api/recipes", HTTP_ANY, handleApiRecipes);
  server.on("/api/command", HTTP_ANY, handleCommandApi);
  server.on("/api/serial", HTTP_GET, handleSerialApi);
  server.on("/api/telemetry", HTTP_GET, handleTelemetryApi);
  server.onNotFound(handleApiRecipeItem);

  server.begin();
  serialPrintlnAndLog("[WebUI] HTTP server started on port 80.");
}

void handle() {
  server.handleClient();
}

void pushSerialLine(const String &line) {
  appendSerialLog(line);
}

} // namespace WebUI
} // namespace App
