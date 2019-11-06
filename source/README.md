# MyPCLTemplate
PointCloudLibraryを扱う際のテンプレート

## はじめかた
```
git clone https://github.com/YoheiSaito/MyPCLTemplate.git
cd MyPCLTemplate
mkdir source
mv .git `ls -a | grep -v source |grep -v "^\." ` source
mkdir build
cd build
cmake ../source/
cp ../source/example.ini .
make
./main example.ini
```
