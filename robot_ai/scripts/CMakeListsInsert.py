##
# Usage: python CMakeListsInsert.py pathToCmakeLists nodeName pkgName [dependencies [dependencies ...]]
# inserts entry for add_executable, add_dependencies, target_link_libraries and
# component dependencies
##


import re
import sys
import fileinput

def stringExists(path, s):
	f = open(path,"r")
	for line in f:
		if re.match("^\\s*"+s+"$", line):
			f.close()
			return True
	f.close()
	return False


def insertAfterFirstOccurrence(f, p, s, skipLines=1):
	insert = True
	match = False
	skipLines -= 1
	skipLines = max(0,skipLines);

	for line in fileinput.input(f, inplace=True):
		if match==False and re.match(p, line):
			match = True
		elif insert==True and match==True:
			if skipLines==0:
				print s 		#insert
				insert=False
			else:
				skipLines-=1
		print line,

def insertComponentRequirement(f, comp):
	p = re.compile('^\\s*find_package\(')
	for line in fileinput.input(f, inplace=True):
		if re.match(p, line):
			if re.search('\\sCOMPONENTS', line)==None:
				print line[:-2]+" COMPONENTS\n)" #strip ) and \n
			else:
				print line,
		else:
			print line,

	insertAfterFirstOccurrence(f, p, "  "+comp)




args = sys.argv[1:];
f = args[0];
node = args[1];
pkg = args[2];

# add component dependencies
p = re.compile('^\\s*find_package\(');
for i in range(3,len(args)):
	#check for redundancy
	if stringExists(f,args[i])==False:
		insertComponentRequirement(f, args[i]);

# add_executable...
p = re.compile('^#?\\s*add_executable\(');
insertAfterFirstOccurrence(f, p, "add_executable("+node+" src/"+node+".cpp)");

# add dependencies...
#p = re.compile('^#?\\s*add_dependencies\(');
#insertAfterFirstOccurrence(f, p, "add_dependencies("+node+" "+pkg+"_generate_messages_cpp)");

#add target_link_libraries...
p = re.compile('^#?\\s*target_link_libraries\(');
insertAfterFirstOccurrence(f, p, "target_link_libraries("+node+"\n  ${catkin_LIBRARIES}\n)",3);
