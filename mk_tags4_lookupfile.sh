#!/bin/bash
# generate tag file for lookupfile plugin

# 用来生成filenametags文件中的”!_TAG_FILE_SORTED”行,
# 表明此tag文件是经过排序的
echo -e "!_TAG_FILE_SORTED\t2\t/2=foldcase/" > filenametags

# 使用find 命令查找文件并排序输出到filenametags
# find 查找语句可根据实际情况定制
# 不包括一些目录
# 忽略.o, .a, .s等后缀的文件
find . -path "./Documentation" -prune -o \
    -path "./arch/[^a]*" -prune -o \
    -path "./arch/alpha" -prune -o \
    -not -regex '.*\.\(o\|a\|s\|so\|ko\|i\|lst\|symtypes\|order\|elf\|bin\|gz\|bz2\|lzma\|patch\|xz\|lzo\|Image\|img\|map\|markers\|symvers\|gitignore\|mailmap\|orig\|bak\|cmd\)' \
    -type f -printf "%f\t%p\t1\n" | \
    sort -f >> filenametags 


# find 语句示例

# 排除.class的所有普通文件
# find . -not -iname "*.class" -type f -printf "%f\t%p\t1\n" \

# find . \( -name .svn -o -wholename ./classes \) -prune -o -not -iregex '.*\.\(jar\|gif\|jpg\|class\|exe\|dll\|pdd\|sw[op]\|xls\|doc\|pdf\|zip\|tar\|ico\|ear\|war\|dat\).*' -type f -printf "%f\t%p\t1\n" \
