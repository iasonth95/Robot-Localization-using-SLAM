# Robot Localization using SLAM
## Configure
cmake -Bbuild -GNinja
## Build
cmake --build build
## Run 
./src/RL-SLAM
### Misc
For better auto completion 
`ln -s build/compile_commands.json compile_commands.json`
and install a clangd extension for your IDE,
e.g. [vscode](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
For some style advices run 
`clang-tidy src/* --checks="-*,modernize-*"`
And follow whatever you agree with, I'll leave them there so you can see them
Also I added some pedantic flags and I left the warnings for you to remove
