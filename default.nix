{ stdenv, lib, cmake, ninja, gitMinimal, protobuf, python312
, west2nix, zephyr, sdk_0_17_0
, zephyr-src
, touchpad-module-src
, board ? "daisy"
, shield ? "daisy"
  # Kconfig fragments merged last, relative to app/.
, extraConfFiles ? [ ]
}:
let
  manifest = lib.importTOML ./west2nix.toml;

  # Private repos are provided via flake inputs (which handle SSH auth).
  # Filter them out of the manifest so the hook only fetches public repos.
  sshRepos = [ "zephyr-private" "zephyr-hid-touchpad-module" ];
  publicManifest = manifest // {
    manifest = manifest.manifest // {
      projects = builtins.filter
        (p: !builtins.elem p.name sshRepos)
        manifest.manifest.projects;
    };
  };

  west2nixHook = west2nix.mkWest2nixHook {
    manifest = publicManifest;
  };

  protobufPython = python312.withPackages (ps: [ ps.protobuf ps.grpcio-tools ]);

  westBuildFlags = [ "-p" "-b" board "--" ]
    ++ lib.optionals (shield != null) [ "-DSHIELD=${shield}" ]
    ++ lib.optionals (extraConfFiles != [ ]) [
      "-DEXTRA_CONF_FILE=${lib.concatStringsSep ";" extraConfFiles}"
    ];
in
stdenv.mkDerivation {
  pname = "zmk-${board}";
  version = "0.1.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = ./app;
  };

  nativeBuildInputs = [
    ((sdk_0_17_0.sdk.override {
      targets = [ "arm-zephyr-eabi" ];
    }).overrideAttrs {
      autoPatchelfIgnoreMissingDeps = [ "libpython3.10.so.1.0" ];
    })
    west2nixHook
    zephyr.pythonEnv
    zephyr.hosttools-nix
    gitMinimal
    cmake
    ninja
    protobuf
  ];

  PYTHONPATH = "${protobufPython}/${python312.sitePackages}";

  dontUseCmakeConfigure = true;

  inherit westBuildFlags;

  # Copy private repos (fetched by flake inputs) into the workspace.
  # At this point we're in the app/ dir (west2nix configureHook did cd).
  preBuild = ''
    __west2nix_copyProject ${zephyr-src} ../zephyr
    __west2nix_copyProject ${touchpad-module-src} ../zephyr-hid-touchpad-module
  '';

  installPhase = ''
    mkdir -p $out
    cp ./build/zephyr/zmk.uf2 $out/ 2>/dev/null || true
    cp ./build/zephyr/zmk.elf $out/
  '';
}
