# Recipe Serial JSON Protocol

This document describes the JSON payloads used by the recipe-related serial
commands and the lightweight client in `tools/sfc_recipe_client.py`.

## Transport

- **Request:** one JSON object per line (newline-terminated).
- **Response:** one JSON object per line.
- **Encoding:** UTF-8.

The firmware responds in the format emitted by `printStructured()`:

```json
{
  "cmd": "sfc.recipe.show",
  "status": "ok",
  "message": "optional message",
  "data": {}
}
```

`status` is `ok` or `error`. `message` is optional. `data` is optional and can be
any JSON object or array.

## Recipe Schema (Util::Recipe)

A recipe is an **array** of steps. Each step is an object with the following
keys:

| Field | Type | Required | Notes |
| --- | --- | --- | --- |
| `volume_ml` | number | yes | Must be `> 0`. |
| `base_slot` | integer | no | 1-based base index. Must be `> 0` if provided. |
| `base_rfid` | integer | no | RFID of the base. Must be `> 0` if provided. |
| `color_hex` | string | no | `#RRGGBB` string. |
| `paint_id` | integer | no | Numeric ID for the paint. Must be `> 0` if provided. |

Each step must include **at least one** identifier (`base_slot`, `base_rfid`,
`color_hex`, or `paint_id`). The maximum number of steps is **32**.

Example recipe:

```json
[
  {"volume_ml": 1.5, "base_slot": 1},
  {"volume_ml": 0.75, "color_hex": "#12ABEF", "paint_id": 42}
]
```

## Commands

### `sfc.recipe.list`

List the stored recipes.

**Request**

```json
{"cmd":"sfc.recipe.list"}
```

**Response data** (example):

```json
{
  "recipes": [
    {"toolhead_rfid": 3735928559},
    {"toolhead_rfid": 305419896}
  ]
}
```

### `sfc.recipe.show`

Show a stored recipe by toolhead RFID.

**Request**

```json
{"cmd":"sfc.recipe.show","data":{"toolhead_rfid":305419896}}
```

**Response data** (example):

```json
{
  "toolhead_rfid": 305419896,
  "recipe": [
    {"volume_ml": 1.5, "base_slot": 1}
  ]
}
```

### `sfc.recipe.save`

Save a recipe for a toolhead RFID.

**Request**

```json
{
  "cmd": "sfc.recipe.save",
  "data": {
    "toolhead_rfid": 305419896,
    "recipe": [
      {"volume_ml": 1.5, "base_slot": 1}
    ]
  }
}
```

**Response data** (example):

```json
{"saved":true}
```

### `sfc.recipe.delete`

Delete a stored recipe by toolhead RFID.

**Request**

```json
{"cmd":"sfc.recipe.delete","data":{"toolhead_rfid":305419896}}
```

**Response data** (example):

```json
{"deleted":true}
```

## CLI Usage

The Python CLI sends the JSON payloads above and prints the JSON response.
Install dependencies first:

```bash
pip install pyserial
```

Examples:

```bash
tools/sfc_recipe_client.py --port /dev/ttyUSB0 list

tools/sfc_recipe_client.py --port /dev/ttyUSB0 show 0x1234ABCD

tools/sfc_recipe_client.py --port /dev/ttyUSB0 save 0x1234ABCD ./recipe.json

tools/sfc_recipe_client.py --port /dev/ttyUSB0 delete 0x1234ABCD
```
