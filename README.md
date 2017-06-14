# Buffer zone for moving files

File stop between transfers.  

## How it works

Computer 1 sits on network A
```
 ______________
||            ||
||            ||
||            ||
||            ||
||____________||
|______________|
 \\############\\
  \\############\\
   \      ____    \   
    \_____\___\____\
```
Computer 2 sits on network B
```
                              \\\\\\\
                            \\\\\\\\\\\\
                          \\\\\\\\\\\\\\\
  -----------,-|           |C>   // )\\\\|
           ,','|          /    || ,'/////|
---------,','  |         (,    ||   /////
         ||    |          \\  ||||//''''|
         ||    |           |||||||     _|
         ||    |______      `````\____/ \
         ||    |     ,|         _/_____/ \
         ||  ,'    ,' |        /          |
         ||,'    ,'   |       |         \  |
_________|/    ,'     |      /           | |
_____________,'      ,',_____|      |    | |
             |     ,','      |      |    | |
             |   ,','    ____|_____/    /  |
             | ,','  __/ |             /   |
_____________|','   ///_/-------------/   |
              |===========,'
```
Development depends on hardware sitting on Computer 1, firmware upload depends on hardware on Computer 2. Repository sits on network B, which Computer 1 does not have access to.  
Workaround, create "sub" git repository in src directory. .gitignore ignores everthing, just add what has been changed. Parent .gitignore in turns ignores .git and .gitignore.  Push changes in src to buffer from Computer 1. From Computer 2, pull changes. src directory uses sames .git and .gitignore, leaving it to parent .git and .gitignore to manage changes.
