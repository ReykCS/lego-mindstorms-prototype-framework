#! /bin/bash

# LEGO PARTS

# Download and unzip lego parts library
PARTS_LINK="https://www.ldraw.org/library/updates/complete.zip"

wget $PARTS_LINK

unzip complete.zip

mv ldraw legoParts

# Copy custom lego parts
cp brickPiParts/brickpi.dat legoParts/parts/brickpi.dat
cp brickPiParts/squarehole.dat legoParts/parts/squarehole.dat

rm complete.zip

git submodule --init --recursive

# LEO CAD

LEO_CAD_LINK="https://github.com/leozide/leocad/releases/download/v21.06/LeoCAD-Linux-21.06-x86_64.AppImage"

cd LeoCAD
wget $LEO_CAD_LINK
cd ..

# LDR TO PROTO

cd ldrToProto 
npm i .
cd ..

# ADD ENV VARIABLES