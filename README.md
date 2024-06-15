# Configure
cmake -Bbuild -GNinja
# Build
cmake --build build
# Run 
./src/localization
coming soon....
# Misc
For better auto completion 
`ln -s build/compile_commands.json compile_commands.json`
and install a clangd extension for your IDE,
e.g. [vscode](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd)
