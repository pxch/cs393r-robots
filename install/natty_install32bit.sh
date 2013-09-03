sudo apt-get install g++-multilib ia32-libs
sudo apt-add-repository ppa:nutznboltz/cappy-getlibs-all
sudo apt-get update
sudo apt-get install getlibs
sudo getlibs -l libqglviewer-qt4.so.2.3.4 libqwt-qt4.so.5.2.0 libQtXml.so.4.7.2 libQtOpenGL.so.4.7.2 libQtGui.so.4.7.2 libQtNetwork.so.4.7.2 libQtCore.so.4.7.2 libGLU.so.1 libpthread.so
sudo getlibs -l libqglviewer-qt4.so libqwt-qt4.so libQtXml.so libQtOpenGL.so libQtGui.so libQtNetwork.so libQtCore.so libGLU.so 
sudo getlibs -p libboost-system-dev libboost-system1.42 libboost-thread-dev libboost-thread1.42

wget http://www.cs.utexas.edu/~sbarrett/share/librcssnet3D.so.0
sudo mkdir -p /usr/lib32/simspark
sudo mv librcssnet3D.so.0 /usr/lib32/simspark/
sudo ln -s /usr/lib32/simspark/librcssnet3D.so.0 /usr/lib32/simspark/librcssnet3D.so
