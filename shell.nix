{ mkShell, cmake, ninja, protobuf, python312, zephyr, sdk_0_17_0 }:
let
  protobufPython = python312.withPackages (ps: [ ps.protobuf ps.grpcio-tools ]);
in
mkShell {
  packages = [
    ((sdk_0_17_0.sdk.override {
      targets = [ "arm-zephyr-eabi" ];
    }).overrideAttrs {
      autoPatchelfIgnoreMissingDeps = [ "libpython3.10.so.1.0" ];
    })
    zephyr.pythonEnv
    zephyr.hosttools-nix
    cmake
    ninja
    protobuf
  ];

  shellHook = ''
    export PYTHONPATH="${protobufPython}/${python312.sitePackages}''${PYTHONPATH:+:$PYTHONPATH}"
  '';
}
