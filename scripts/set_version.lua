function update_version(package_xml, version)
    local file = io.open(package_xml, "r"):read("a")
    file = file:gsub("(<version>).-(</version>)",
        "%1"..version.."%2")
    io.open(package_xml, "w+"):write(file)
end

print("Setting version to "..arg[2].." in "..arg[1])
update_version(arg[1], arg[2])