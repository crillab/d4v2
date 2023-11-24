#!/bin/bash

# Check if running from within MSYS2.
if [ -z "$MSYSTEM" ];
then
    >&2 echo "This is intended to be run from within an MSYS2 environment."
    exit 1
fi

# Check arguments for correct length.
if [ "$#" -lt 2 ]; then
    >&2 echo "Usage: $0 <destination> [<path to executable/library>]"
    exit 1
fi

# Setup variables from environment.
system=$(echo "$MSYSTEM" | tr '[:upper:]' '[:lower:]')
destination=$1
files=${@:2}

# Check for MSYS2 environment to be present.
if [ ! -d /"$MSYSTEM" ];
then
    >&2 echo "Path of MSYS2 $MSYSTEM environment not found."
    exit 1
fi

# Check for destination to be present.
if [ ! -d "$destination" ];
then
    >&2 echo "Destination does not exist."
    exit 1
fi

# Check for all (to be processed) files to be present.
for file in $files
do
    if [ ! -f "$file" ];
    then
        >&2 echo "No such file: $file"
        exit 1
    fi
done

# Generate list of needed packages.
packages=()

for file in $files
do
    dlls=$(ldd "$file" | grep /"$system" | sed 's/.dll.*/.dll/')

    for dll in $dlls
    do
        package=$(pacman -Qqo "$dll")
        echo "[Library]:  $dll ($package)"
        packages+=("$package")
    done
done

# Remove duplicates.
packages=$(echo "${packages[@]}" | tr ' ' '\n' | sort -u)

echo "[Required]: $(echo "$packages" | tr '\n' ' ')"

# Extract packages.
cwd=$PWD
tmp=$(mktemp -d)
cd "$tmp" || exit 1

for package in $packages
do
    path=$(pacman -Sp "$package" | grep -oP 'file://\K\S+')
    echo "[Extract]:  $package ($path)"
    tar -xf "$path"
done

# Copy files to destination.
cp -r "$tmp/$system/bin" "$destination"
cp -r "$tmp/$system/share/licenses" "$destination"

# Cleanup.
cd "$cwd" || exit 1
rm -r "$tmp"
