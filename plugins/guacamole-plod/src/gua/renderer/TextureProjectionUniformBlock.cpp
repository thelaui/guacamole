#include <gua/renderer/TextureProjectionUniformBlock.hpp>

namespace gua {

TextureProjectionUniformBlock::TextureProjectionUniformBlock(scm::gl::render_device_ptr const& device)
{
  uniform_block_ = scm::gl::make_uniform_block<TextureProjectionBlock>(device);
}

TextureProjectionUniformBlock::~TextureProjectionUniformBlock()
{
  uniform_block_.reset();
}

void TextureProjectionUniformBlock::update(RenderContext const& context,
                                           std::vector<math::mat4f> const& projection_view_mats,
                                           std::vector<math::vec2ui> const& projection_textures) {

  uniform_block_.begin_manipulation(context.render_context); {
    for (unsigned i(0); i < MAX_PROJECTIVE_TEXTURE_COUNT && i < projection_view_mats.size(); ++i) {
      uniform_block_->projection_view_mats[i] = projection_view_mats[i];
    }
    for (unsigned i(0); i < MAX_PROJECTIVE_TEXTURE_COUNT && i < projection_textures.size(); ++i) {
      uniform_block_->projection_textures[i] = projection_textures[i];
    }
    uniform_block_->frustum_count = std::min(int(projection_view_mats.size()), MAX_PROJECTIVE_TEXTURE_COUNT);
  } uniform_block_.end_manipulation();
}

} // namespace gua {
