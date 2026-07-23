find ~/rpi5_cross/sysroot/usr/lib/aarch64-linux-gnu -name "*.so.*" | while read f; do
  base=$(basename "$f")
  nover=${base%%.so.*}.so
  dir=$(dirname "$f")
  if [ ! -e "$dir/$nover" ]; then
    echo "Création du lien $nover -> $base"
    ln -s "$base" "$dir/$nover"
  fi
done
