cd simAdapter/
./gen_swig.sh
cd ../
./premake4_linux clean
./premake4_linux gmake

cd gmake
make config=release64
cd ../