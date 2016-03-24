#!/bin/bash


url="http://www.github.com/abcminiuser/lufa/archive/LUFA-151115.zip"
zipfile="${url##*/}"
basename="lufa-${zipfile%$'.zip'}"
tmpDir="lufatmp"
os=$(uname -s)

if [ ! -e LUFA/Version.h ]; then
  if [ ! -e "${zipfile}" ]; then
    wget "${url}"
  fi

  unzip -q "${zipfile}" ${basename}/LUFA/* -d "${tmpDir}"
  mv "${tmpDir}/${basename}/LUFA" LUFA
  \rm -rf "${tmpDir}"
#  \rm "${zipfile}"

  echo -n "Extracted "
else
  echo -n "Found "
fi

# print out the installed version
grep -H LUFA_VERSION_STRING LUFA/Version.h | awk '{print $1, $2, $3, $4 }'
