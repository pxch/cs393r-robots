sudo apt-get install g++-multilib ia32-libs #g++-4.5 g++-4.5-multilib
#wget http://frozenfox.freehostia.com/cappy/getlibs-all.deb
wget http://www.cs.utexas.edu/~sbarrett/share/getlibs-all.deb
sudo dpkg -i getlibs-all.deb
rm getlibs-all.deb
sudo getlibs -l libqglviewer-qt4.so.2.3.4 libqwt-qt4.so.5.2.2 libQtXml.so.4.8.1 libQtOpenGL.so.4.8.1 libQtGui.so.4.8.1 libQtNetwork.so.4.8.1 libQtCore.so.4.8.1 libGLU.so.1 libpthread.so
sudo getlibs -l libqglviewer-qt4.so libqwt-qt4.so libQtXml.so libQtOpenGL.so libQtGui.so libQtNetwork.so libQtCore.so libGLU.so 
sudo getlibs -l libicui18n.so libicui18n.so.48.1.1 libappmenu.so libdbusmenu-gtk.so.4 libdbusmenu-glib.so.4
sudo getlibs -p libboost-system1.48-dev libboost-system1.48.1 libboost-thread1.48-dev libboost-thread1.48.1

wget http://www.cs.utexas.edu/~sbarrett/share/librcssnet3D.so.0
sudo mv librcssnet3D.so.0 /usr/lib/i386-linux-gnu/
sudo ln -s /usr/lib/i386-linux-gnu/librcssnet3D.so.0 /usr/lib/i386-linux-gnu/librcssnet3D.so
./install_python
