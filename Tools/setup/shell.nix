let
	pkgs = import (builtins.fetchTarball {
		name = "nixos-20.09-2020-10-29";
		url = "https://github.com/nixos/nixpkgs/archive/edb26126d98bc696f4f3e206583faa65d3d6e818.tar.gz";
		sha256 = "1cl4ka4kk7kh3bl78g06dhiidazf65q8miyzaxi9930d6gwyzkci";
	}) {};
	empy = with pkgs.python3Packages; buildPythonPackage rec {
		pname = "empy";
		version = "3.3.4";
		src = fetchPypi {
			inherit pname version;
			sha256 = "1cq1izl6l87i5i3vj0jcqfksh10kpiwpr2m19vgpj530bdw4kb3k";
		};
		doCheck = false;
	};
	pyros-genmsg = with pkgs.python3Packages; buildPythonPackage rec {
		pname = "pyros-genmsg";
		version = "0.5.8";
		src = fetchPypi {
			inherit version;
			pname = "pyros_genmsg";
			sha256 = "0y7l131lc77v0c1rhxza41cxnnxc7acfqzlqf84fdya0kiyv071w";
		};
		doCheck = false;
	};
	pyulog = with pkgs.python3Packages; buildPythonPackage rec {
		pname = "pyulog";
		version = "0.8.0";
		src = fetchPypi {
			inherit pname version;
			sha256 = "1ivvhfi9rsrqdk9f06rj0q1d367ngyy0xyc2x9mdwjx3dazwgn45";
		};
		propagatedBuildInputs = [ numpy ];
		doCheck = false;
	};
in pkgs.mkShell {
	nativeBuildInputs = [ pkgs.cmake ];
	buildInputs = [
		pkgs.gcc-arm-embedded
		pkgs.python3
	] ++ (with pkgs.python3Packages; [
		argcomplete
		cerberus
		coverage
		empy
		jinja2
		matplotlib
		numpy
		packaging
		pandas
		pkgconfig
		psutil
		pygments
		pyros-genmsg
		pyserial
		pyulog
		pyyaml
		requests
		setuptools
		six
		toml
		wheel
	]);
}
