/**
 * @file WebUI.cpp
 * @brief Embedded HTTP server for recipe CRUD and a tiny UI.
 */
#include "app/WebUI.hpp"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebServer.h>

#include <WifiCredentials.hpp>
#include <WifiManager.hpp>

#include "util/Recipe.hpp"
#include "util/Storage.hpp"

namespace {

WebServer server(80);

constexpr size_t kMaxRecipeList = 64;
Shared::WifiManager g_wifi;

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
  <script>
    const listEl = document.getElementById('recipeList');
    const stepsEl = document.getElementById('steps');
    const recipeIdEl = document.getElementById('recipeId');
    const statusEl = document.getElementById('status');

    function setStatus(msg, ok = true) {
      statusEl.textContent = msg;
      statusEl.style.color = ok ? '#2b6' : '#c33';
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

    refreshList();
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

void handleListRecipes() {
  Serial.printf("[WebUI] GET /api/recipes\n");
  uint32_t ids[kMaxRecipeList];
  size_t count = 0;
  if (!Util::listRecipeIds(ids, kMaxRecipeList, count)) {
    Serial.println("[WebUI] listRecipeIds() failed");
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
  Serial.printf("[WebUI] GET /api/recipes/%08X\n", recipeId);
  Util::Recipe recipe;
  if (!Util::loadRecipe(recipeId, recipe)) {
    Serial.printf("[WebUI] loadRecipe() failed for %08X\n", recipeId);
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
  Serial.printf("[WebUI] PUT /api/recipes/%08X\n", recipeId);
  if (!server.hasArg("plain")) {
    Serial.println("[WebUI] PUT missing body");
    server.send(400, "text/plain", "Missing body");
    return;
  }

  Serial.printf("[WebUI] PUT payload: %s\n", server.arg("plain").c_str());
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    Serial.printf("[WebUI] PUT JSON parse error: %s\n", err.c_str());
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
    Serial.println("[WebUI] PUT missing steps array");
    server.send(400, "text/plain", "Missing steps array");
    return;
  }

  Util::Recipe recipe;
  recipe.fromJson(arr);
  if (recipe.isEmpty()) {
    Serial.println("[WebUI] PUT recipe invalid or empty after parsing");
    server.send(400, "text/plain", "Recipe invalid or empty");
    return;
  }

  Serial.printf("[WebUI] PUT parsed %u step(s)\n", recipe.count);
  for (uint8_t i = 0; i < recipe.count; ++i) {
    Serial.printf("[WebUI] step %u: volume_ml=%.3f base_slot=%d\n",
                  i, recipe.steps[i].volumeMl, recipe.steps[i].baseSlot);
  }
  if (!Util::saveRecipe(recipeId, recipe)) {
    Serial.printf("[WebUI] saveRecipe() failed for %08X\n", recipeId);
    server.send(500, "text/plain", "Save failed");
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleDeleteRecipe(uint32_t recipeId) {
  Serial.printf("[WebUI] DELETE /api/recipes/%08X\n", recipeId);
  if (!Util::deleteRecipe(recipeId)) {
    Serial.printf("[WebUI] deleteRecipe() failed for %08X\n", recipeId);
    server.send(404, "text/plain", "Delete failed");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleApiRecipes() {
  Serial.printf("[WebUI] %s /api/recipes\n", server.method() == HTTP_GET ? "GET" : "OTHER");
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
    Serial.printf("[WebUI] Not found: %s\n", uri.c_str());
    server.send(404, "text/plain", "Not found");
    return;
  }

  String hexStr = uri.substring(prefix.length());
  uint32_t recipeId = 0;
  if (!parseRecipeId(hexStr, recipeId)) {
    Serial.printf("[WebUI] Invalid recipe ID from URI: %s\n", uri.c_str());
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
    Serial.printf("[WebUI] Method not allowed for %s\n", uri.c_str());
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
    Serial.println("[WebUI] Falling back to AP mode.");
  } else {
    Serial.println("[WebUI] No saved WiFi credentials found.");
  }
  g_wifi.startAccessPoint();
  Serial.println("[WebUI] Open http://192.168.4.1/ to configure.");
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
  server.onNotFound(handleApiRecipeItem);

  server.begin();
  Serial.println("[WebUI] HTTP server started on port 80.");
}

void handle() {
  server.handleClient();
}

} // namespace WebUI
} // namespace App
