#!/bin/bash

CONFIG="config/parameters_processing.yaml"
RAW_DIR="data/raw"
PCD_DIR="data/pcd"
TILES_DIR="data/tiles"

# Načti výstupní formát z YAML konfigurace
SAVE_FORMAT=$(
  grep -Po '(?<=save_format: ).*' "$CONFIG" \
  | tr -d '"' \
  | tr '[:upper:]' '[:lower:]' \
  | tr -d $'\r' \
  | xargs
)
echo "🧪 SAVE_FORMAT='$SAVE_FORMAT'"

# 1. Převod LAS → PCD pomocí CloudCompare Dockeru
echo "🔁 Převádím LAS → PCD pomocí CloudCompare Dockeru..."
mkdir -p "$PCD_DIR"

for file in "$RAW_DIR"/*.las; do
    [ -e "$file" ] || continue
    fname=$(basename "$file" .las)
    echo "➡️  $fname.las → $fname.pcd (via CloudCompare)"

    docker run --rm -v "$(pwd)/data:/data" cloudcompare-cli \
        -SILENT -AUTO_SAVE OFF \
        -O "/data/raw/$fname.las" \
        -C_EXPORT_FMT PCD \
        -NO_TIMESTAMP \
        -SAVE_CLOUDS FILE "/data/pcd/$fname.pcd"
done

echo "✅ Převod LAS → PCD dokončen."

# 2. Spuštění pipeline (čištění, augmentace, dlaždice)
echo "🚀 Spouštím pipeline v Dockeru..."
docker-compose up

# 3. PCD → LAS/LAZ pokud je požadovaný výstup
if [[ "$SAVE_FORMAT" == "laz" || "$SAVE_FORMAT" == "las" ]]; then
    echo "🔁 Převádím tiled PCD → $SAVE_FORMAT pomocí CloudCompare Dockeru..."
    for file in "$TILES_DIR"/*.pcd; do
        [ -e "$file" ] || continue
        fname=$(basename "$file" .pcd)
        input="/data/tiles/$fname.pcd"
        output="/data/tiles/$fname.$SAVE_FORMAT"

        echo "➡️  $fname.pcd → $fname.$SAVE_FORMAT"

        docker run --rm -v "$(pwd)/data:/data" cloudcompare-cli \
            -SILENT -AUTO_SAVE OFF \
            -O "$input" \
            -C_EXPORT_FMT "${SAVE_FORMAT^^}" \
            -NO_TIMESTAMP \
            -SAVE_CLOUDS FILE "$output"
    done
    echo "✅ Převod tiled PCD → $SAVE_FORMAT dokončen."
else
    echo "💡 Výstupní formát je $SAVE_FORMAT → žádná konverze tiles se neprovádí."
fi
