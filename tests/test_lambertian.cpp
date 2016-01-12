#include "catch.hpp"

#include <assimp/mesh.h>

TEST_CASE("Lambertian", "[light]") {
    GIVEN("A Face on a Mesh with one face") {
        aiMesh mesh;
        mesh.mNumFaces = 1;
        mesh.mFaces = new aiFace[1];
        FAIL("TODO: Write some tests.");
        REQUIRE(false);
    }
}
