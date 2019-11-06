# MyPCLTemplate
PointCloudLibraryを扱う際のテンプレート

## はじめかた
```
git clone https://github.com/YoheiSaito/MyPCLTemplate.git
cd MyPCLTemplate
mkdir source
mv `ls -a | grep -v -e source -e \. -e \.\.` source
mkdir build
cd build
cmake ../source/
cp ../source/example.ini .
make
```
