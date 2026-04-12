#!/usr/bin/env python3
"""
Parse ELRS Lua script and generate menu.json for the repeater
Usage: python3 parse_elrs_lua.py sdcard/scripts/elrs.lua sdcard/menu.json
"""

import re
import json
import sys
from pathlib import Path

def parse_elrs_lua(lua_file):
    """Parse elrs.lua and extract menu structure"""
    with open(lua_file, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    menu = {
        "version": "1.0",
        "items": []
    }
    
    # Extract field definitions - look for type definitions
    # This is a simplified parser focusing on key structures
    
    # Look for function table definitions showing field types:
    # Type 0: UINT8, 1: INT8, 2: UINT16, 3: INT16, 8: FLOAT, 9: SELECT (textsel), 10: STRING, 11: FOLDER, 13: COMMAND, 14: BACK, 15: device, 16: deviceFOLDER
    
    type_names = {
        0: "uint8",
        1: "int8", 
        2: "uint16",
        3: "int16",
        8: "float",
        9: "select",
        10: "string",
        11: "folder",
        12: "info",
        13: "command",
        14: "back",
        15: "device",
        16: "device_folder"
    }
    
    # Try to extract basic information from comments
    # The ELRS Lua has lots of info, but extracting would require full parser
    # For now, create a template that user can fill in
    
    # Extract version if available
    match = re.search(r'local EXITVER = "-- EXIT \(Lua r(\d+)\) --"', content)
    if match:
        menu["lua_version"] = f"r{match.group(1)}"
    
    # Extract any hardcoded device references
    device_ids = re.findall(r'0x([0-9A-Fa-f]+)', content)
    
    # Generate a documented template
    menu["documentation"] = {
        "description": "TX menu structure for ELRS repeater",
        "type_reference": {
            "0": "UINT8",
            "1": "INT8",
            "2": "UINT16", 
            "3": "INT16",
            "8": "FLOAT",
            "9": "SELECT (with options list)",
            "10": "STRING",
            "11": "FOLDER",
            "12": "INFO (read-only)",
            "13": "COMMAND (executable)",
            "14": "BACK/EXIT",
            "15": "DEVICE (selectable)",
            "16": "DEVICE FOLDER"
        },
        "instructions": [
            "Edit the 'items' array below to define your TX menu",
            "Each item needs: id, name, type, and type-specific fields",
            "parent: folder id (0 = root, null/omit for root items)",
            "For select type: add 'options' array with choices",
            "For numeric: add min, max, step, unit fields",
            "Load this file at ESP startup to populate TX menu"
        ]
    }
    
    # Create example structure based on common ELRS fields
    menu["items"] = [
        {
            "id": 1,
            "name": "Power",
            "type": 0,
            "parent": None,
            "min": 10,
            "max": 250,
            "step": 10,
            "unit": "mW",
            "description": "TX RF Power in milliwatts"
        },
        {
            "id": 2,
            "name": "Rate",
            "type": 9,
            "parent": None,
            "options": ["50 Hz", "150 Hz", "250 Hz"],
            "description": "Packet transmission rate"
        },
        {
            "id": 3,
            "name": "UART",
            "type": 11,
            "parent": None,
            "description": "UART/Serial Settings",
            "isFolder": True
        },
        {
            "id": 31,
            "name": "Baud Rate",
            "type": 9,
            "parent": 3,
            "options": ["115200", "250000", "420000", "460800"],
            "description": "Serial baud rate"
        },
        {
            "id": 100,
            "name": "Save Config",
            "type": 13,
            "parent": None,
            "description": "Save current settings to device"
        }
    ]
    
    return menu

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 parse_elrs_lua.py <elrs.lua> [output.json]")
        sys.exit(1)
    
    lua_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "menu.json"
    
    if not Path(lua_file).exists():
        print(f"Error: {lua_file} not found")
        sys.exit(1)
    
    print(f"Parsing {lua_file}...")
    menu = parse_elrs_lua(lua_file)
    
    # Write output
    with open(output_file, 'w') as f:
        json.dump(menu, f, indent=2)
    
    print(f"Generated {output_file}")
    print(f"Lua version: {menu.get('lua_version', 'unknown')}")
    print(f"Menu items: {len(menu['items'])}")

if __name__ == "__main__":
    main()
