from pathlib import Path


def test_read_version_info(cobra, mock):
    p = Path("/etc", "lumotive_fs_rev")
    version_message = "Not available"
    if mock:
        assert cobra.compute.os_build_number == version_message
        assert cobra.compute.os_build_sha == version_message
        assert cobra.compute.os_build_version == version_message
        assert cobra.compute.manifest == version_message
        assert cobra.compute.manifest_sha == version_message
    else:
        if p.is_file():
            with p.open('r', encoding='utf8') as f:
                j = f.readline().strip().split("=")
                assert cobra.compute.os_build_sha == j[1]
                k = f.readline().strip().split("=")
                assert cobra.compute.os_build_version == k[1]
                l = f.readline().strip().split("=")
                assert cobra.compute.os_build_number == l[1]
                m = f.readline().strip().split("=")
                assert cobra.compute.manifest == m[1]
                n = f.readline().strip().split("=")
                assert cobra.compute.manifest_sha == n[1]
        else:
            assert cobra.compute.os_build_number == version_message
            assert cobra.compute.os_build_sha == version_message
            assert cobra.compute.os_build_version == version_message
            assert cobra.compute.manifest == version_message
            assert cobra.compute.manifest_sha == version_message
