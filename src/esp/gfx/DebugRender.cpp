// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DebugRender.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/CubicHermite.h>
#include "Magnum/Animation/Interpolation.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

DebugRender::DebugRender(const std::size_t initialBufferCapacity)
    : _mesh{Mn::GL::MeshPrimitive::Lines} {
  _mesh.addVertexBuffer(_buffer, 0, Mn::Shaders::VertexColor3D::Position{},
                        Mn::Shaders::VertexColor3D::Color4{});
  arrayReserve(_bufferData, initialBufferCapacity * 2);  // todo: what is this?
}

void DebugRender::drawLine(const Mn::Vector3& from,
                           const Mn::Vector3& to,
                           const Mn::Color4& color) {
  drawLine(from, to, color, color);
}

void DebugRender::drawLine(const Mn::Vector3& from,
                           const Mn::Vector3& to,
                           const Mn::Color4& fromColor,
                           const Mn::Color4& toColor) {
  VertexRecord v1{from, fromColor};
  VertexRecord v2{to, toColor};
  arrayAppend(_bufferData, {v1, v2});
}

void DebugRender::flushLines() {
  bool doToggleBlend = !glIsEnabled(GL_BLEND);

  if (doToggleBlend) {
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::Blending);
  }

  // Update buffer with new data
  _buffer.setData(_bufferData, Mn::GL::BufferUsage::DynamicDraw);

  // Update shader
  _mesh.setCount(_bufferData.size());
  _shader.setTransformationProjectionMatrix(_transformationProjectionMatrix);

  // draw normal
  _shader.draw(_mesh);

  // modify all colors to be semi-transparent
  // perf todo: do a custom shader constant for opacity instead so we don't have
  // to touch all the verts
  constexpr float opacity = 0.25;
  for (int v = 0; v < _bufferData.size(); v++) {
    _bufferData[v].color.w() *= opacity;
  }
  _buffer.setData(_bufferData, Mn::GL::BufferUsage::DynamicDraw);

  Mn::GL::Renderer::setDepthFunction(Mn::GL::Renderer::DepthFunction::Greater);

  _shader.draw(_mesh);

  Mn::GL::Renderer::setDepthFunction(Mn::GL::Renderer::DepthFunction::Less);

  /* Clear buffer to receive new data */
  arrayResize(_bufferData, 0);

  if (doToggleBlend) {
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::Blending);
  }
}

void DebugRender::pushInputTransform(const Magnum::Matrix4& transform) {
  _inputTransformStack.push_back(transform);
  updateCachedInputTransform();
}

void DebugRender::popInputTransform() {
  CORRADE_INTERNAL_ASSERT(!_inputTransformStack.empty());
  _inputTransformStack.pop_back();
  updateCachedInputTransform();
}

void DebugRender::drawTransformedLine(const Magnum::Vector3& from,
                                      const Magnum::Vector3& to,
                                      const Magnum::Color4& color) {
  drawTransformedLine(from, to, color, color);
}

void DebugRender::drawTransformedLine(const Magnum::Vector3& from,
                                      const Magnum::Vector3& to,
                                      const Magnum::Color4& fromColor,
                                      const Magnum::Color4& toColor) {
  Mn::Vector3 fromTransformed = _cachedInputTransform.transformPoint(from);
  Mn::Vector3 toTransformed = _cachedInputTransform.transformPoint(to);
  drawLine(fromTransformed, toTransformed, fromColor, toColor);
}

void DebugRender::drawBox(const Magnum::Vector3& min,
                          const Magnum::Vector3& max,
                          const Magnum::Color4& color) {
  // 4 lines along x axis
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), min.z()),
                      Mn::Vector3(max.x(), min.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), max.z()),
                      Mn::Vector3(max.x(), min.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), max.y(), min.z()),
                      Mn::Vector3(max.x(), max.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), max.y(), max.z()),
                      Mn::Vector3(max.x(), max.y(), max.z()), color);

  // 4 lines along y axis
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), min.z()),
                      Mn::Vector3(min.x(), max.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), min.y(), min.z()),
                      Mn::Vector3(max.x(), max.y(), min.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), max.z()),
                      Mn::Vector3(min.x(), max.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), min.y(), max.z()),
                      Mn::Vector3(max.x(), max.y(), max.z()), color);

  // 4 lines along z axis
  drawTransformedLine(Mn::Vector3(min.x(), min.y(), min.z()),
                      Mn::Vector3(min.x(), min.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), min.y(), min.z()),
                      Mn::Vector3(max.x(), min.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(min.x(), max.y(), min.z()),
                      Mn::Vector3(min.x(), max.y(), max.z()), color);
  drawTransformedLine(Mn::Vector3(max.x(), max.y(), min.z()),
                      Mn::Vector3(max.x(), max.y(), max.z()), color);
}

void DebugRender::drawCircle(const Magnum::Vector3& pos,
                             const Magnum::Vector3& normal,
                             float radius,
                             int numSegments,
                             const Magnum::Color4& color) {
  // https://stackoverflow.com/questions/11132681/what-is-a-formula-to-get-a-vector-perpendicular-to-another-vector
  auto randomPerpVec = Mn::Math::abs(normal.z()) < Mn::Math::abs(normal.x())
                           ? Mn::Vector3(normal.y(), -normal.x(), 0)
                           : Mn::Vector3(0, -normal.z(), normal.y());

  pushInputTransform(Mn::Matrix4::lookAt(pos, pos + normal, randomPerpVec) *
                     Mn::Matrix4::scaling(Mn::Vector3(radius, radius, 0.f)));

  Mn::Vector3 prevPt;
  for (int seg = 0; seg <= numSegments; seg++) {
    Mn::Deg angle = Mn::Deg(360.f * float(seg) / numSegments);
    Mn::Vector3 pt(Mn::Math::cos(angle), Mn::Math::sin(angle), 0.f);
    if (seg > 0) {
      drawTransformedLine(prevPt, pt, color);
    }
    prevPt = pt;
  }

  popInputTransform();
}

void DebugRender::drawCurve(
    const std::vector<Magnum::Vector3>& controlPositions,
    const std::vector<Magnum::Vector3>& controlTangents,
    const Magnum::Color4& color,
    int numSegmentsPerControl) {
  CORRADE_INTERNAL_ASSERT(controlPositions.size() >= 2);
  CORRADE_INTERNAL_ASSERT(controlPositions.size() == controlTangents.size());

  Mn::Vector3 prevPt;
  for (int cp = 1; cp < controlPositions.size(); cp++) {
    Mn::CubicHermite3D a{controlTangents[cp - 1], controlPositions[cp - 1],
                         controlTangents[cp - 1]};
    Mn::CubicHermite3D b{controlTangents[cp - 0], controlPositions[cp - 0],
                         controlTangents[cp - 0]};
    for (int segment = 0; segment <= numSegmentsPerControl; segment++) {
      if (cp > 1 && segment == 0) {
        continue;  // already computed as last point of previous segment
      }
      const float fraction = (float)segment / numSegmentsPerControl;
      Mn::Vector3 pt = Mn::Animation::interpolatorFor<Mn::CubicHermite3D>(
          Mn::Animation::Interpolation::Spline)(a, b, fraction);
      if (cp > 1 || segment > 0) {
        drawTransformedLine(prevPt, pt, color);
      }
      prevPt = pt;
    }
  }
};

void DebugRender::updateCachedInputTransform() {
  _cachedInputTransform = Mn::Matrix4{Magnum::Math::IdentityInit};
  for (const auto& item : _inputTransformStack) {
    _cachedInputTransform = _cachedInputTransform * item;
  }
}

}  // namespace gfx
}  // namespace esp
