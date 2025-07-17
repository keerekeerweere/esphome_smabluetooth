def align_down(value, alignment):
    return value & ~(alignment - 1)

def generate_partition_csv(flash_size_bytes=0x400000, useCoreDump=False):
    """
    Generates a valid ESP32 partition table CSV string for a given flash size (default: 4MB).
    Ensures app0/app1 start and size are aligned to 0x10000 (64KB).
    Optionally includes coredump.
    """
    ALIGNMENT = 0x10000  # 64KB

    # Fixed system partitions
    partitions = [
        ("nvs", "data", "nvs", 0x9000, 0x5000),
        ("otadata", "data", "ota", 0xE000, 0x2000),
    ]

    app0_start = 0x10000

    # Reserve space for EEPROM, SPIFFS, and optional core dump
    eeprom_size = 0x1000
    spiffs_size = 0x400
    coredump_size = 0x4000 if useCoreDump else 0

    reserved_end = eeprom_size + spiffs_size + coredump_size
    available_space = flash_size_bytes - app0_start - reserved_end

    # Align available app partition space down to multiple of ALIGNMENT * 2 (for even app split)
    total_app_space = align_down(available_space, ALIGNMENT * 2)
    app_size = total_app_space // 2
    app1_start = app0_start + app_size

    if useCoreDump:
        coredump_start = app1_start + app_size
        eeprom_start = coredump_start + coredump_size
    else:
        eeprom_start = app1_start + app_size

    spiffs_start = eeprom_start + eeprom_size

    # App and data partitions
    partitions += [
        ("app0", "app", "ota_0", app0_start, app_size),
        ("app1", "app", "ota_1", app1_start, app_size),
    ]

    if useCoreDump:
        partitions.append(("coredump", "data", "coredump", coredump_start, coredump_size))

    partitions += [
        ("eeprom", "data", "0x99", eeprom_start, eeprom_size),
        ("spiffs", "data", "spiffs", spiffs_start, spiffs_size),
    ]

    # Format CSV: both hex (for ESPHome) and decimal (for tooling like esptool.py)
    lines = [f"{name}, {ptype}, {subtype}, 0x{start:X}, 0x{size:X}" for name, ptype, subtype, start, size in partitions]
    lines += ["", "# Decimal version"]
    lines += [f"{name}, {ptype}, {subtype}, {start}, {size}" for name, ptype, subtype, start, size in partitions]

    return "\n".join(lines)

if __name__ == "__main__":
    csv = generate_partition_csv()
    print(csv)
