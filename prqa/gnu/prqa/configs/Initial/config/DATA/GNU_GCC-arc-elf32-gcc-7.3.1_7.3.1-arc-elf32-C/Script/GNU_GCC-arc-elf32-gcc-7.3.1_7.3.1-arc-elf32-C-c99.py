import os, inspect
import sys

def move_to_top(f, fs):
  if f in fs:
    i = fs.index(f)
    fs.pop(i)
    fs.insert(0, f)
  return fs

# add include paths for CCT stub directories
def stub():
  try:
    scriptFile = inspect.getfile(inspect.currentframe())
    currentDir = os.path.dirname(os.path.abspath(scriptFile))
    fileName = os.path.basename(scriptFile)
    cipFilename = os.path.splitext(fileName.strip())[0]
    normal_parent = os.path.dirname(currentDir)
    isCPP = os.path.basename(normal_parent).endswith("C++")
    dataDir = os.path.dirname(normal_parent)
    includeOption = None
    if isCPP:
      includeOption = "-si"
    else:
      includeOption = "-i"
    cipFilename = cipFilename + ".cip"
    cctDir = os.path.dirname(dataDir)
    configDir = os.path.dirname(cctDir)
    cipDir = os.path.join(configDir, "cip")
    cipFilepath = os.path.join(cipDir, cipFilename)
    stubDir = os.path.join(normal_parent, "Stub")
    cipFile = open(cipFilepath, 'w')

    # stub directory
    for d, ds, fs in os.walk(stubDir):
      if d.endswith("prlforceinclude"):
        sorted = fs
        move_to_top("builtins_all.h", sorted)
        move_to_top("qacpp_builtin.h", sorted)
        move_to_top("prlg++.h", sorted)
        move_to_top("prlgcc.h", sorted) # this ends up at the top
        for fn in sorted:
          print >>cipFile, "-fi", "\"" + os.path.join(d, fn) + "\""
      else:
        shortName = d[len(stubDir):]
        sepCount = shortName.count(os.sep)
        if (sepCount == 1 and len(fs) != 0) or (sepCount > 1 and len(ds) != 0):
          path = "\"" + d + "\""
          print >>cipFile, includeOption, path
          print >>cipFile, "-q", path
    # add paths from syshdr.lst
    print >>cipFile, "* system include paths"
    templatePath = os.path.join(stubDir, 'syshdr.lst')
    dirFile = open(templatePath, "r")
    root = ""
    expectRoot = False
    for sysPath in dirFile:
      sysPath = sysPath.strip()
      if expectRoot:
        expectRoot = False
        root = sysPath
      elif sysPath == ":ROOT:":
        expectRoot = True
      else:
        path = "\"" +  root + sysPath + "\""
        print >>cipFile, includeOption, path
        print >>cipFile, "-q", path
    dirFile.close()
    cipFile.close()
  except IOError:
    pass

if __name__=="__main__":
  stub()

