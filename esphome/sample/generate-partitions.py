def align_up(value, alignment):
    return (value + alignment - 1) & ~(alignment - 1)

def generate_partition_csv(flash_size_bytes=0x400000, useCoreDump=False):
    """
    Generate a valid ESP32 partition table, ensuring total size does not exceed flash size.
    Aligns app partitions to 0x10000 (64KB), and checks cumulative size.
    """
    ALIGNMENT = 0x10000  # 64KB
    partitions = []

    total = 0

    # Fixed system partitions
    def add(name, ptype, subtype, start, size):
        nonlocal total
        total = max(total, start + size)
        partitions.append((name, ptype, subtype, start, size))

    add("nvs", "data", "nvs", 0x9000, 0x5000)
    add("otadata", "data", "ota", 0xE000, 0x2000)

    app0_start = align_up(0x10000, ALIGNMENT)

    # Reserve space at end
    eeprom_size = 0x1000
    spiffs_size = 0x1000
    coredump_size = 0x4000 if useCoreDump else 0

    reserved_end = eeprom_size + spiffs_size + coredump_size
    available_space = flash_size_bytes - app0_start - reserved_end

    total_app_space = (available_space // (2 * ALIGNMENT)) * ALIGNMENT * 2
    app_size = total_app_space // 2

    app1_start = app0_start + app_size

    add("app0", "app", "ota_0", app0_start, app_size)
    add("app1", "app", "ota_1", app1_start, app_size)

    if useCoreDump:
        coredump_start = app1_start + app_size
        add("coredump", "data", "coredump", coredump_start, coredump_size)
    else:
        coredump_start = app1_start + app_size

    eeprom_start = coredump_start if not useCoreDump else coredump_start + coredump_size
    spiffs_start = eeprom_start + eeprom_size

    add("eeprom", "data", "0x99", eeprom_start, eeprom_size)
    add("spiffs", "data", "spiffs", spiffs_start, spiffs_size)

    # Final overflow check
    if total > flash_size_bytes:
        raise ValueError(f"Partition layout exceeds flash size! Used: {hex(total)}, Limit: {hex(flash_size_bytes)}")

    # Print total
    print(f"# Total size used: {total} bytes = {total / 1024:.1f} KB = {total / (1024*1024):.2f} MB")

    # Output both hex and decimal
    lines = [f"{name}, {ptype}, {subtype}, 0x{start:X}, 0x{size:X}" for name, ptype, subtype, start, size in partitions]
    lines += ["", "# Decimal version"]
    lines += [f"{name}, {ptype}, {subtype}, {start}, {size}" for name, ptype, subtype, start, size in partitions]

    return "\n".join(lines)

if __name__ == "__main__":
    print(generate_partition_csv(flash_size_bytes=0x400000, useCoreDump=False))
