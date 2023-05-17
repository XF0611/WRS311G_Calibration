# coding: utf-8

import os
import sys
from xml.etree import ElementTree as etree


CURRENT_DIR = os.path.abspath(os.path.dirname(__file__))
SRC_DIR = os.path.dirname(os.path.dirname(CURRENT_DIR))

cip = '''-i "{src_dir}/prqa/gnu/prqa/configs/Initial/config/DATA/GNU_GCC-arc-elf32-gcc-7.3.1_7.3.1-arc-elf32-C/Stub/gcc"
-q "{src_dir}/prqa/gnu/prqa/configs/Initial/config/DATA/GNU_GCC-arc-elf32-gcc-7.3.1_7.3.1-arc-elf32-C/Stub/gcc"
-fi "{src_dir}/prqa/gnu/prqa/configs/Initial/config/DATA/GNU_GCC-arc-elf32-gcc-7.3.1_7.3.1-arc-elf32-C/Stub/prlforceinclude/prlgcc.h"
* system include paths
-i "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/include"
-q "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/include"
-i "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/include-fixed"
-q "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/include-fixed"
-i "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/../../../../arc-elf32/sys-include"
-q "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/../../../../arc-elf32/sys-include"
-i "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/../../../../arc-elf32/include"
-q "{arc_gnu}/lib/gcc/arc-elf32/7.3.1/../../../../arc-elf32/include"
'''

lst = '''{arc_gnu}/lib/gcc/arc-elf32/7.3.1/include
{arc_gnu}/lib/gcc/arc-elf32/7.3.1/include-fixed
{arc_gnu}/lib/gcc/arc-elf32/7.3.1/../../../../arc-elf32/sys-include
{arc_gnu}/lib/gcc/arc-elf32/7.3.1/../../../../arc-elf32/include
'''

def main(arc_gnu, src_dir):
    arc_gnu = os.path.abspath(arc_gnu)
    src_dir = os.path.abspath(src_dir)

    cip_path = os.path.join(CURRENT_DIR, 'prqa/configs/Initial/cip/GNU_GCC-arc-elf32-gcc-7.3.1_7.3.1-arc-elf32-C-c99.cip')
    with open(cip_path, 'w') as f:
        s = cip.format(src_dir=src_dir, arc_gnu=arc_gnu).replace('/', os.sep).replace('\\', os.sep)
        f.write(s)

    lst_path = os.path.join(CURRENT_DIR, 'prqa/configs/Initial/config/DATA/GNU_GCC-arc-elf32-gcc-7.3.1_7.3.1-arc-elf32-C/Stub/syshdr.lst')
    with open(lst_path, 'w') as f:
        s = lst.format(arc_gnu=arc_gnu).replace('/', os.sep).replace('\\', os.sep)
        f.write(s)

    app_path = os.path.join(CURRENT_DIR, 'prqa/qa-framework-app.xml')
    tree = etree.parse(app_path)
    elem = tree.find(".//setting[@name='sync_working_directory']")
    elem.set('value', src_dir)
    tree.write(app_path, encoding="UTF-8", xml_declaration=True)

    proj_path = os.path.join(CURRENT_DIR, 'prqaproject.xml')
    tree = etree.parse(proj_path)
    elem = tree.find(".//root_path[@name='SOURCE_ROOT']")
    elem.set('path', src_dir)
    tree.write(proj_path, encoding="UTF-8", xml_declaration=True)

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument('--arc-gnu', required=True)
    parser.add_argument('--src-dir', default=SRC_DIR)
    args = parser.parse_args()

    main(args.arc_gnu, args.src_dir)
