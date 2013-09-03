#!/usr/bin/env python

import sys, os, shutil, subprocess

def runRemoteCommand(ip,command):
  subprocess.check_call('ssh nao@%s "%s"' % (ip,command),shell=True)

def copyDirectory(ip,filename,dest=''):
  subprocess.check_call('scp -r %s nao@%s:%s' % (filename,ip,dest),shell=True)

def main(robotIP):
  home = os.environ["NAO_HOME"]
  runRemoteCommand(robotIP,'mkdir -p ~/python2.7')
  copyDirectory(robotIP,'%s/naoqi/crosstoolchain/atom/sysroot/lib/python2.7' % home,'~')
  print "use 'root' for the next prompt"
  subprocess.check_call('ssh -t -t nao@%s "su -c \'rm -rf /lib/python2.7 && mv python2.7 /lib && echo export LD_LIBRARY_PATH=/home/nao/bin >> /etc/profile\'"' % robotIP,shell=True)

if __name__ == '__main__':
 usage = 'setup_python robotIP'
 helpStrings = ['-h', '--help']
 args = sys.argv[1:]
 for s in helpStrings:
  if s in args:
    print usage
    sys.exit(0)
 robotIP = args[0]
 main(robotIP)
