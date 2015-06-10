#ifndef GUA_TEXTURE_PROJECTION_UNIFORM_BLOCK_HPP
#define GUA_TEXTURE_PROJECTION_UNIFORM_BLOCK_HPP

#include <gua/math/math.hpp>
#include <gua/renderer/RenderContext.hpp>

#include <scm/gl_core/render_device/render_device_fwd.h>
#include <scm/gl_core/buffer_objects/uniform_buffer_adaptor.h>

#define MAX_PROJECTIVE_TEXTURE_COUNT 300

namespace gua {

class TextureProjectionUniformBlock
{
public:
  struct TextureProjectionBlock {
    math::mat4f  projection_view_mats[MAX_PROJECTIVE_TEXTURE_COUNT];
    math::vec2ui projection_textures[MAX_PROJECTIVE_TEXTURE_COUNT];
    int          frustum_count;
  };

  typedef scm::gl::uniform_block<TextureProjectionBlock> block_type;

  TextureProjectionUniformBlock(scm::gl::render_device_ptr const& device);
  ~TextureProjectionUniformBlock();

  void update(RenderContext const& context,
              std::vector<math::mat4f> const& projection_view_mats,
              std::vector<math::vec2ui> const& projection_textures);

  inline const block_type&   block() const { return uniform_block_; }

private:
  block_type   uniform_block_;
};

} // namespace gua {

#endif // #ifndef GUA_TEXTURE_PROJECTION_UNIFORM_BLOCK_HPP
