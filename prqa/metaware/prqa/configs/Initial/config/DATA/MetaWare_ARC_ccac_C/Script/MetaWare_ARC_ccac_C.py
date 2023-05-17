import os, sys, inspect, imp
import dircache
import fnmatch
import dircache

def sysInclude(myCct):
    try:
        # walk through stub directory and add them to cip
        cipFile = open(myCct.cipFilePath(), 'w')
        stubList = [os.path.join(d, t) for d, ds, fs in os.walk(myCct.stubDir()) for t in ds]
        for name in stubList:
            cipFile.write('-i "' + name + '"\n')
            cipFile.write('-q "' + name + '"\n')
        for fileName in stubList:
            if fileName.endswith("forceinclude"):
                fileList = [os.path.join(fileName, x) for x in myCct.getForceIncludes(fileName)]
                for fn in fileList:
                    cipFile.write('-fi "' + fn + '"\n')
        cipFile.write('-i "' + 'C:\\ARC\\MetaWare\\arc\\inc' + '"\n')
        cipFile.write('-q "' + 'C:\\ARC\\MetaWare\\arc\\inc' + '"\n')
        cipFile.write('-i "' + 'C:\\ARC\\MetaWare\\arc\\incc' + '"\n')
        cipFile.write('-q "' + 'C:\\ARC\\MetaWare\\arc\\incc' + '"\n')
        cipFile.close()

    except IOError:
        myCct.exit_error(1, 'Error writing to CIP', cipFile)

def compileInclude():
    cipFile = open(myCct.cipFilePath(), 'w')
    cipFile.write('-i "' + 'C:\\ARC\\MetaWare\\arc\\inc' + '"\n')
    cipFile.write('-q "' + 'C:\\ARC\\MetaWare\\arc\\inc' + '"\n')
    cipFile.write('-i "' + 'C:\\ARC\\MetaWare\\arc\\incc' + '"\n')
    cipFile.write('-q "' + 'C:\\ARC\\MetaWare\\arc\\incc' + '"\n')
    cipFile.close()

if __name__=="__main__":
    qafDir = os.path.abspath(
        os.path.join(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))), "..", "..", "core_qaf"))
    if qafDir not in sys.path:
        sys.path.insert(0, qafDir)
    try:
        imp.find_module('qaf')
    except ImportError:
        print >> sys.stderr, 'Unable to locate "qaf" module. Exiting.'
        sys.exit(1)
    import qaf
    myCct = qaf.Cip(os.path.abspath(inspect.getfile(inspect.currentframe())))
    sysInclude(myCct)

    '''compileInclude() '''