#!/usr/bin/env python

import os, shutil, subprocess

def runRemoteCommand(ip,command):
  subprocess.check_call('ssh nao@%s "%s"' % (ip,command),shell=True)

def copyFile(ip,filename,dest=''):
  subprocess.check_call('scp %s nao@%s:%s' % (filename,ip,dest),shell=True)

def copyFileAndMove(ip,filename,dest):
  tmpDest = os.path.basename(filename) + '.tmp'
  copyFile(ip,filename,tmpDest)
  return 'mv %s %s\n' % (tmpDest,dest)

def setHostname(hostname):
  return 'echo %s > /etc/hostname\n' % hostname

def removeTempDir(d):
  shutil.rmtree(d)

def makeTempName():
  d = os.path.join(os.path.expandvars('$NAO_HOME/install/tmp'))
  return d

def makeTempDir(d):
  os.mkdir(d)
  return d

def filloutID(name,robotID,d):
  with open('../data/scripts/%s' % name,'r') as f:
    contents = f.read()
  contents = contents.replace('${ID}',robotID)
  with open(os.path.join(d,name),'w') as f:
    f.write(contents)

def filloutField(name,field,d):
  with open('../data/scripts/%s' % name,'r') as f:
    contents = f.read()
  contents = contents.replace('${ID}',robotID)
  with open(os.path.join(d,name),'w') as f:
    f.write(contents)

def setupNetworking(ip,robotID,d,hostname):#,field):
  filloutID('utwired',robotID,d)
  filloutID('utwireless',robotID,d)
  #filloutField('wpa_supplicant.conf',field,d)

  script = '#!/bin/bash\n\n'
  script += copyFileAndMove(ip,'../data/scripts/wpa_supplicant.conf','/etc/wpa_supplicant/wpa_supplicant.conf')
  #script += copyFileAndMove(ip,os.path.join(d,'wpa_supplicant.conf'),'/etc/wpa_supplicant/wpa_supplicant.conf')
  script += copyFileAndMove(ip,os.path.join(d,'utwired'),'/etc/init.d/utwired')
  script += copyFileAndMove(ip,os.path.join(d,'utwireless'),'/etc/init.d/utwireless')
  script += 'chmod +x /etc/init.d/utwired\n'
  script += 'chmod +x /etc/init.d/utwireless\n'
  if hostname is not None:
    script += setHostname(hostname)
  script += '/etc/init.d/utwireless stop\n'
  script += '/etc/init.d/utwireless start\n'

  scriptName = 'setupScript.sh'
  scriptPath = os.path.join(d,scriptName)
  

  with open(scriptPath,'w') as f:
    f.write(script)
  copyFile(ip,scriptPath)
  print 'Moving a bunch of stuff around, removing connman, etc.'
  print 'You need to put in the password "root" after it prompts for the password'
  subprocess.check_call('ssh -t -t nao@%s "su -c \'bash %s\'"' % (ip,scriptName),shell=True)
  runRemoteCommand(ip,'rm %s' % scriptName)

def main(ip,robotID,atom,hostname):#,field=None):
  #if field is None:
    #field = 'SPL_FIELD_A'
  d = makeTempName()
  try:
    makeTempDir(d)
    print 'setting up networking'
    setupNetworking(ip,robotID,d,hostname)#,field)
  finally:
    removeTempDir(d)

if __name__ == '__main__':
  import sys
  usage = 'setup_wireless currentIP robotID hostname [--geode]\n'
  usage += 'ex: setup_wireless 11.0.1.37 37 cheesy'
  helpStrings = ['-h','--help']
  args = sys.argv[1:]
  for s in helpStrings:
    if s in args:
      print usage
      sys.exit(0)
  atom = None
  if len(args) < 2:
    print usage
    sys.exit(1)
  currentIP = args[0]
  robotID = args[1]
  hostname = None
  if len(args) >= 3:
    hostname = args[2]
  #field = None
  #if len(args) > 3:
    #field = args[3]
  main(currentIP,robotID,atom,hostname)#,field)
