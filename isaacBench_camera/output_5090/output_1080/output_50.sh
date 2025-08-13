[Info] [carb] Logging to file: /home/dvangelder/.cache/packman/chk/kit-kernel/107.3.1+isaac.206797.8131b85d.gl.manylinux_2_35_x86_64.release/logs/Kit/Isaac-Sim Python/5.0/kit_20250810_194735.log
[0.055s] [ext: omni.kit.async_engine-0.0.3] startup
[0.681s] [ext: omni.metrics.core-0.0.3] startup
[0.681s] [ext: omni.client.lib-1.1.0] startup
[0.694s] [ext: omni.blobkey-1.1.2] startup
[0.694s] [ext: omni.stats-1.0.1] startup
[0.695s] [ext: omni.datastore-0.0.0] startup
[0.700s] [ext: omni.client-1.3.0] startup
[0.754s] [ext: omni.ujitso.default-1.0.0] startup
[0.757s] [ext: omni.hsscclient-1.1.2] startup
[0.761s] [ext: omni.gpu_foundation.shadercache.vulkan-1.0.0] startup
[0.764s] [ext: omni.assets.plugins-0.0.0] startup
[0.765s] [ext: omni.gpu_foundation-0.0.0] startup
[0.772s] [ext: carb.windowing.plugins-1.0.0] startup
[0.779s] [ext: omni.kit.renderer.init-0.0.0] startup
2025-08-11T02:47:36Z [1,053ms] [Warning] [gpu.foundation.plugin] Skipping unsupported non-NVIDIA GPU: AMD Unknown (RADV RAPHAEL_MENDOCINO)
2025-08-11T02:47:36Z [1,053ms] [Warning] [gpu.foundation.plugin] Skipping unsupported non-NVIDIA GPU: AMD Unknown (RADV RAPHAEL_MENDOCINO)

|---------------------------------------------------------------------------------------------|
| Driver Version: 570.172.08    | Graphics API: Vulkan
|=============================================================================================|
| GPU | Name                             | Active | LDA | GPU Memory | Vendor-ID | LUID       |
|     |                                  |        |     |            | Device-ID | UUID       |
|     |                                  |        |     |            | Bus-ID    |            |
|---------------------------------------------------------------------------------------------|
| 0   | NVIDIA GeForce RTX 5090          | Yes: 0 |     | 32607   MB | 10de      | 0          |
|     |                                  |        |     |            | 2b85      | 3da6d810.. |
|     |                                  |        |     |            | 1         |            |
|---------------------------------------------------------------------------------------------|
| 1   | AMD Unknown (RADV RAPHAEL_MEND.. |        |     | 32259   MB | 1002      | 0          |
|     |                                  |        |     |            | 164e      | 00000000.. |
|     |                                  |        |     |            | c         |            |
|=============================================================================================|
| OS: 22.04.5 LTS (Jammy Jellyfish) ubuntu, Version: 22.04.5, Kernel: 6.8.0-65-generic
| XServer Vendor: The X.Org Foundation, XServer Version: 12101004 (1.21.1.4)
| Processor: AMD Ryzen 9 7950X3D 16-Core Processor
| Cores: 16 | Logical Cores: 32
|---------------------------------------------------------------------------------------------|
| Total Memory (MB): 95753 | Free Memory: 81168
| Total Page/Swap (MB): 2047 | Free Page/Swap: 2047
|---------------------------------------------------------------------------------------------|
2025-08-11T02:47:36Z [1,322ms] [Warning] [gpu.foundation.plugin] CPU performance profile is set to powersave. This profile sets the CPU to the lowest frequency reducing performance.
2025-08-11T02:47:36Z [1,327ms] [Warning] [gpu.foundation.plugin] IOMMU is enabled.
2025-08-11T02:47:36Z [1,327ms] [Warning] [gpu.foundation.plugin] Detected IOMMU is enabled. Running CUDA peer-to-peer bandwidth and latency validation.
Unidirectional P2P=Enabled Bandwidth (P2P Writes) Matrix (GB/s)
   D\D     0 
     0 1511.12 
P2P=Enabled Latency (P2P Writes) Matrix (us)
   GPU     0 
     0   2.12 

   CPU     0 
     0   1.57 
[1.512s] [ext: omni.kit.pipapi-0.0.0] startup
[1.514s] [ext: omni.kit.pip_archive-0.0.0] startup
[1.514s] [ext: omni.materialx.libs-1.0.7] startup
[1.520s] [ext: omni.pip.compute-1.6.2] startup
[1.520s] [ext: omni.usd.config-1.0.6] startup
[1.523s] [ext: omni.gpucompute.plugins-0.0.0] startup
[1.524s] [ext: omni.usd.libs-1.0.1] startup
[1.586s] [ext: omni.mdl-56.0.3] startup
[1.603s] [ext: omni.iray.libs-0.0.0] startup
[1.608s] [ext: omni.mdl.neuraylib-0.2.12] startup
[1.610s] [ext: omni.kit.usd.mdl-1.1.4] startup
[1.680s] [ext: omni.pip.cloud-1.3.6] startup
[1.683s] [ext: omni.isaac.core_archive-2.6.1] startup
[1.684s] [ext: omni.kit.telemetry-0.5.2] startup
[1.710s] [ext: omni.kit.loop-isaac-1.3.7] startup
[1.712s] [ext: omni.kit.test-2.0.1] startup
[1.757s] [ext: omni.isaac.ml_archive-3.0.2] startup
[1.757s] [ext: omni.appwindow-1.1.10] startup
[1.759s] [ext: omni.kit.renderer.core-1.1.0] startup
[1.828s] [ext: omni.kit.renderer.capture-0.0.0] startup
[1.830s] [ext: omni.kit.renderer.imgui-2.0.5] startup
[1.888s] [ext: omni.ui-2.26.18] startup
[1.897s] [ext: omni.kit.mainwindow-1.0.3] startup
[1.899s] [ext: carb.audio-0.1.0] startup
[1.904s] [ext: omni.uiaudio-1.0.0] startup
[1.904s] [ext: omni.kit.uiapp-0.0.0] startup
[1.904s] [ext: omni.usd.schema.metrics.assembler-107.3.1] startup
[1.907s] [ext: omni.usd.schema.anim-0.0.0] startup
[1.921s] [ext: omni.usd.schema.omnigraph-1.0.0] startup
[1.924s] [ext: omni.usd.schema.geospatial-0.0.0] startup
[1.926s] [ext: omni.anim.graph.schema-107.3.0] startup
[1.931s] [ext: omni.usd.schema.audio-0.0.0] startup
[1.933s] [ext: omni.usd.schema.semantics-0.0.0] startup
[1.935s] [ext: omni.usd_resolver-1.0.0] startup
[1.937s] [ext: usdrt.scenegraph-7.6.1] startup
[1.965s] [ext: omni.kit.actions.core-1.0.0] startup
[1.967s] [ext: omni.timeline-1.0.14] startup
[1.968s] [ext: omni.usd.core-1.5.3] startup
[1.970s] [ext: omni.resourcemonitor-107.0.1] startup
[1.972s] [ext: omni.activity.core-1.0.3] startup
[1.973s] [ext: omni.kit.audiodeviceenum-1.0.2] startup
[1.974s] [ext: omni.hydra.usdrt_delegate-7.5.1] startup
[1.979s] [ext: omni.graph.exec-0.9.6] startup
[1.979s] [ext: omni.kit.window.popup_dialog-2.0.24] startup
[1.982s] [ext: omni.kit.exec.core-0.13.4] startup
[1.983s] [ext: omni.kit.widget.nucleus_connector-2.0.1] startup
[1.985s] [ext: omni.kit.usd_undo-0.1.8] startup
[1.986s] [ext: omni.kit.commands-1.4.10] startup
[1.988s] [ext: omni.hydra.scene_delegate-0.3.4] startup
[1.992s] [ext: omni.usd-1.13.10] startup
2025-08-11T02:47:37Z [1,998ms] [Warning] [omni.usd.audio] failed to subscribe to stage events
[2.024s] [ext: omni.inspect-1.0.2] startup
[2.025s] [ext: omni.kit.notification_manager-1.0.10] startup
[2.026s] [ext: omni.graph.core-2.184.3] startup
[2.029s] [ext: isaacsim.core.deprecation_manager-0.2.7] startup
[2.030s] [ext: omni.usd.schema.omniscripting-1.0.0] startup
[2.033s] [ext: omni.usd.schema.isaac-3.0.5] startup
[2.033s] [ext: isaacsim.robot.schema-3.5.1] startup
[2.038s] [ext: omni.usd.schema.omni_sensors-0.0.0] startup
[2.039s] [ext: omni.usd.schema.omni_lens_distortion-0.0.0] startup
[2.040s] [ext: omni.anim.navigation.schema-107.3.0] startup
[2.042s] [ext: omni.usd.schema.physx-107.3.18] startup
[2.067s] [ext: omni.usd.schema.flow-107.1.1] startup
[2.067s] [ext: omni.usd.schema.render_settings.rtx-0.0.0] startup
[2.068s] [ext: omni.kit.asset_converter-4.1.4] startup
[2.076s] [ext: omni.convexdecomposition-107.3.18] startup
[2.078s] [ext: omni.graph.tools-1.79.2] startup
[2.098s] [ext: omni.physx.foundation-107.3.18] startup
[2.099s] [ext: omni.kvdb-107.3.18] startup
[2.100s] [ext: omni.graph.action_core-1.1.7] startup
[2.102s] [ext: omni.graph-1.141.2] startup
[2.135s] [ext: omni.usdphysics-107.3.18] startup
[2.136s] [ext: omni.localcache-107.3.18] startup
[2.138s] [ext: omni.kit.menu.core-1.1.2] startup
[2.139s] [ext: omni.kit.widget.path_field-2.0.11] startup
[2.140s] [ext: omni.kit.widget.options_menu-1.1.6] startup
[2.143s] [ext: omni.kit.widget.context_menu-1.2.5] startup
[2.144s] [ext: omni.kit.widget.browser_bar-2.0.10] startup
[2.145s] [ext: omni.kit.clipboard-1.0.5] startup
[2.146s] [ext: omni.kit.widget.options_button-1.0.3] startup
[2.147s] [ext: omni.kit.helper.file_utils-0.1.9] startup
[2.148s] [ext: omni.kit.usd.layers-2.2.10] startup
[2.154s] [ext: omni.kit.menu.utils-2.0.3] startup
[2.161s] [ext: omni.ui.scene-1.11.4] startup
[2.164s] [ext: omni.kit.widget.filebrowser-2.12.2] startup
[2.168s] [ext: omni.kit.stage_template.core-1.1.22] startup
[2.169s] [ext: omni.kit.primitive.mesh-1.0.17] startup
[2.172s] [ext: omni.kit.window.filepicker-2.13.3] startup
[2.184s] [ext: omni.kit.stage_templates-2.0.0] startup
[2.186s] [ext: omni.graph.action_nodes-1.50.4] startup
[2.192s] [ext: omni.physx.cooking-107.3.18] startup
[2.196s] [ext: omni.warp.core-1.7.1] startup
[2.338s] [ext: omni.usd.metrics.assembler-107.3.1] startup
[2.353s] [ext: omni.physx-107.3.18] startup
[2.364s] [ext: omni.physics-107.3.18] startup
[2.367s] [ext: isaacsim.core.experimental.utils-0.2.0] startup
[2.368s] [ext: omni.graph.action-1.130.0] startup
[2.369s] [ext: omni.kit.manipulator.viewport-107.0.1] startup
[2.371s] [ext: omni.physics.physx-107.3.18] startup
[2.372s] [ext: omni.physics.stageupdate-107.3.18] startup
[2.374s] [ext: omni.kit.numpy.common-0.1.2] startup
[2.375s] [ext: omni.kit.property.adapter.core-1.0.2] startup
[2.377s] [ext: isaacsim.test.docstring-1.1.0] startup
[2.382s] [ext: omni.isaac.dynamic_control-2.0.7] startup
2025-08-11T02:47:37Z [2,368ms] [Warning] [omni.isaac.dynamic_control] omni.isaac.dynamic_control is deprecated as of Isaac Sim 4.5. No action is needed from end-users.
[2.386s] [ext: omni.hydra.rtx.shadercache.vulkan-1.0.0] startup
[2.388s] [ext: omni.hydra.engine.stats-1.0.3] startup
[2.389s] [ext: omni.physics.tensors-107.3.18] startup
[2.394s] [ext: omni.index.libs-380600.5717.0] startup
[2.395s] [ext: omni.volume-0.5.2] startup
[2.397s] [ext: omni.ujitso.client-0.0.0] startup
[2.397s] [ext: omni.index-1.0.1] startup
[2.398s] [ext: omni.usd.metrics.assembler.physics-107.3.18] startup
[2.399s] [ext: omni.hydra.rtx-1.0.0] startup
2025-08-11T02:47:37Z [2,396ms] [Warning] [omni.log] Source: omni.hydra was already registered.
[2.427s] [ext: omni.kit.viewport.registry-104.0.6] startup
[2.428s] [ext: omni.kit.viewport.legacy_gizmos-1.0.18] startup
[2.429s] [ext: omni.kit.raycast.query-1.1.0] startup
[2.433s] [ext: omni.kit.context_menu-1.8.6] startup
[2.435s] [ext: omni.kit.viewport.scene_camera_model-1.0.6] startup
[2.438s] [ext: omni.kit.hydra_texture-1.4.5] startup
[2.440s] [ext: omni.kit.window.file_importer-1.1.18] startup
[2.442s] [ext: isaacsim.core.version-2.0.6] startup
[2.443s] [ext: omni.kit.window.drop_support-1.0.5] startup
[2.444s] [ext: omni.kit.widget.viewport-107.1.3] startup
[2.448s] [ext: omni.kit.material.library-2.0.7] startup
[2.452s] [ext: isaacsim.storage.native-1.2.8] startup
[2.455s] [ext: omni.kit.viewport.window-107.0.12] startup
[2.470s] [ext: omni.physx.tensors-107.3.18] startup
[2.473s] [ext: isaacsim.core.utils-3.4.5] startup
[2.475s] [ext: omni.kit.viewport.utility-1.1.2] startup
[2.477s] [ext: omni.kit.menu.create-2.0.1] startup
[2.479s] [ext: isaacsim.core.simulation_manager-1.3.2] startup
[3.676s] [ext: omni.graph.visualization.nodes-2.1.3] startup
[3.681s] [ext: omni.kit.property.adapter.usd-1.0.2] startup
[3.683s] [ext: omni.kit.property.adapter.fabric-1.0.3] startup
[3.686s] [ext: omni.kit.widget.searchable_combobox-1.0.6] startup
[3.688s] [ext: omni.kit.usd.collect-2.4.5] startup
[3.693s] [ext: omni.kit.window.file_exporter-1.0.33] startup
[3.694s] [ext: omni.kit.hotkeys.core-1.3.10] startup
[3.696s] [ext: omni.kit.widget.highlight_label-1.0.3] startup
[3.697s] [ext: omni.kit.window.file-2.0.5] startup
[3.700s] [ext: omni.kit.widget.searchfield-1.1.8] startup
[3.701s] [ext: omni.kit.widget.filter-1.1.4] startup
[3.701s] [ext: omni.kit.window.content_browser_registry-0.0.6] startup
[3.703s] [ext: omni.kit.widget.stage_icons-1.0.8] startup
[3.703s] [ext: omni.kit.widget.stage-3.1.4] startup
[3.713s] [ext: omni.kit.window.property-1.12.1] startup
[3.716s] [ext: omni.kit.window.content_browser-3.1.1] startup
[3.724s] [ext: omni.kit.window.stage-2.6.1] startup
[3.727s] [ext: omni.kit.property.usd-4.5.11] startup
[3.736s] [ext: omni.ui_query-1.1.8] startup
[3.737s] [ext: omni.kit.widget.prompt-1.0.7] startup
[3.738s] [ext: omni.kit.ui_test-1.3.6] startup
[3.740s] [ext: omni.kit.tool.collect-2.2.18] startup
[3.742s] [ext: omni.kit.scripting-107.3.1] startup
[3.745s] [ext: isaacsim.core.experimental.prims-0.6.2] startup
[3.765s] [ext: omni.kit.widget.zoombar-1.0.6] startup
[3.767s] [ext: omni.kit.window.extensions-1.4.26] startup
[3.772s] [ext: isaacsim.replicator.behavior-1.1.14] startup
[3.773s] [ext: isaacsim.core.experimental.objects-0.2.4] startup
[3.777s] [ext: omni.kit.browser.core-2.3.13] startup
[3.781s] [ext: omni.kit.usdz_export-1.0.9] startup
[3.784s] [ext: omni.kit.menu.stage-1.2.7] startup
[3.785s] [ext: omni.kit.browser.folder.core-1.10.9] startup
[3.789s] [ext: omni.kit.tool.asset_importer-4.3.2] startup
[3.793s] [ext: isaacsim.gui.components-1.1.9] startup
[3.797s] [ext: isaacsim.examples.browser-0.1.12] startup
[3.801s] [ext: isaacsim.asset.importer.urdf-2.4.19] startup
[3.845s] [ext: isaacsim.simulation_app-2.9.2] startup
[3.846s] [ext: omni.graph.ui_nodes-1.50.5] startup
[3.853s] [ext: omni.isaac.kit-2.0.6] startup
2025-08-11T02:47:39Z [3,836ms] [Warning] [omni.isaac.kit] omni.isaac.kit has been deprecated in favor of isaacsim.simulation_app. Please update your code accordingly.
[3.854s] [ext: omni.kit.widget.graph-2.0.0] startup
[3.862s] [ext: omni.graph.image.core-0.6.1] startup
[3.863s] [ext: omni.kit.widget.settings-1.2.3] startup
[3.865s] [ext: omni.kit.graph.delegate.default-1.2.2] startup
[3.866s] [ext: omni.graph.image.nodes-1.3.1] startup
[3.867s] [ext: omni.kit.window.preferences-1.8.0] startup
[3.879s] [ext: omni.kit.graph.editor.core-1.5.3] startup
[3.883s] [ext: omni.kit.graph.usd.commands-1.3.1] startup
[3.885s] [ext: omni.graph.nodes-1.170.10] startup
[3.892s] [ext: omni.kit.viewport.menubar.core-107.1.5] startup
[3.906s] [ext: omni.kit.widget.material_preview-1.0.16] startup
[3.908s] [ext: omni.videoencoding-0.1.2] startup
[3.909s] [ext: isaacsim.core.prims-0.5.1] startup
[3.923s] [ext: omni.warp-1.7.1] startup
[3.929s] [ext: omni.syntheticdata-0.6.13] startup
[3.958s] [ext: omni.kit.window.material_graph-1.8.23] startup
[4.028s] [ext: omni.graph.scriptnode-1.50.0] startup
[4.031s] [ext: isaacsim.core.api-4.6.10] startup
[4.045s] [ext: omni.replicator.core-1.12.16] startup
2025-08-11T02:47:39Z [4,035ms] [Warning] [pxr.Semantics] pxr.Semantics is deprecated - please use Semantics instead
2025-08-11T02:47:39Z [4,116ms] [Warning] [omni.replicator.core.scripts.extension] No material configuration file, adding configuration to material settings directly.
[4.136s] [ext: isaacsim.robot_motion.lula-4.0.8] startup
[4.144s] [ext: isaacsim.robot.surface_gripper-3.2.6] startup
[4.147s] [ext: isaacsim.core.nodes-3.2.16] startup
[4.161s] [ext: isaacsim.robot_motion.motion_generation-8.0.25] startup
[4.177s] [ext: isaacsim.robot.manipulators-3.3.5] startup
[4.186s] [ext: omni.isaac.core_nodes-2.0.6] startup
2025-08-11T02:47:39Z [4,173ms] [Warning] [omni.isaac.core_nodes] omni.isaac.core_nodes has been deprecated in favor of isaacsim.core.nodes. Please update your code accordingly.
[4.193s] [ext: isaacsim.robot.manipulators.examples-1.0.17] startup
[4.197s] [ext: omni.kit.property.audio-1.0.16] startup
[4.204s] [ext: isaacsim.cortex.framework-1.0.12] startup
[4.206s] [ext: omni.kit.property.camera-1.0.10] startup
[4.212s] [ext: omni.hydra.scene_api-0.1.2] startup
[4.221s] [ext: omni.kit.property.geometry-2.0.4] startup
[4.227s] [ext: omni.kit.property.light-1.0.12] startup
[4.243s] [ext: isaacsim.cortex.behaviors-2.0.12] startup
[4.244s] [ext: omni.kit.property.material-1.11.9] startup
[4.259s] [ext: omni.kit.property.transform-1.5.12] startup
[4.266s] [ext: omni.kit.property.render-1.2.1] startup
[4.268s] [ext: omni.isaac.cortex-1.0.5] startup
2025-08-11T02:47:39Z [4,252ms] [Warning] [omni.isaac.cortex] omni.isaac.cortex has been deprecated in favor of isaacsim.cortex.framework. Please update your code accordingly.
[4.269s] [ext: omni.isaac.cortex.sample_behaviors-2.0.5] startup
2025-08-11T02:47:39Z [4,254ms] [Warning] [omni.isaac.cortex.sample_behaviors] omni.isaac.cortex.sample_behaviors has been deprecated in favor of isaacsim.cortex.behaviors. Please update your code accordingly.
[4.272s] [ext: omni.anim.curve.core-1.3.1] startup
[4.291s] [ext: isaacsim.robot_motion.lula_test_widget-1.0.12] startup
[4.294s] [ext: omni.kit.property.bundle-1.4.1] startup
[4.302s] [ext: omni.isaac.lula-4.0.6] startup
2025-08-11T02:47:39Z [4,288ms] [Warning] [omni.isaac.lula] omni.isaac.lula has been deprecated in favor of isaacsim.robot_motion.lula. Please update your code accordingly.
[4.314s] [ext: omni.isaac.manipulators-3.0.7] startup
2025-08-11T02:47:39Z [4,299ms] [Warning] [omni.isaac.manipulators] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,306ms] [Warning] [omni.isaac.manipulators.controllers] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,307ms] [Warning] [omni.isaac.manipulators.controllers.pick_place_controller] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,308ms] [Warning] [omni.isaac.manipulators.controllers.stacking_controller] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,309ms] [Warning] [omni.isaac.manipulators.grippers] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,309ms] [Warning] [omni.isaac.manipulators.grippers.gripper] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,310ms] [Warning] [omni.isaac.manipulators.grippers.parallel_gripper] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,311ms] [Warning] [omni.isaac.manipulators.grippers.surface_gripper] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,311ms] [Warning] [omni.isaac.manipulators.manipulators] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
2025-08-11T02:47:39Z [4,311ms] [Warning] [omni.isaac.manipulators.manipulators.single_manipulator] omni.isaac.manipulators has been deprecated in favor of isaacsim.robot.manipulators. Please update your code accordingly.
[4.331s] [ext: omni.kit.viewport.menubar.lighting-107.3.1] startup
[4.348s] [ext: omni.isaac.lula_test_widget-1.0.6] startup
2025-08-11T02:47:39Z [4,331ms] [Warning] [omni.isaac.lula_test_widget] omni.isaac.lula_test_widget has been deprecated in favor of isaacsim.robot_motion.lula_test_widget. Please update your code accordingly.
[4.349s] [ext: isaacsim.core.throttling-2.1.10] startup
[4.354s] [ext: isaacsim.asset.importer.mjcf-2.5.8] startup
[4.370s] [ext: omni.kit.selection-0.1.6] startup
[4.374s] [ext: omni.kit.widget.toolbar-2.0.1] startup
[4.457s] [ext: omni.kit.manipulator.transform-107.0.0] startup
[4.501s] [ext: isaacsim.gui.menu-2.3.17] startup
[4.653s] [ext: isaacsim.asset.browser-1.3.19] startup
[4.659s] [ext: omni.kit.manipulator.tool.snap-1.5.13] startup
[4.664s] [ext: omni.kit.manipulator.selector-1.1.3] startup
[4.666s] [ext: omni.isaac.menu-1.0.6] startup
2025-08-11T02:47:40Z [4,648ms] [Warning] [omni.isaac.menu] omni.isaac.menu has been deprecated in favor of isaacsim.gui.menu. Please update your code accordingly.
[4.666s] [ext: omni.isaac.motion_generation-8.0.7] startup
2025-08-11T02:47:40Z [4,650ms] [Warning] [omni.isaac.motion_generation] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,651ms] [Warning] [omni.isaac.motion_generation.articulation_kinematics_solver] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,652ms] [Warning] [omni.isaac.motion_generation.articulation_motion_policy] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,652ms] [Warning] [omni.isaac.motion_generation.articulation_trajectory] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,652ms] [Warning] [omni.isaac.motion_generation.kinematics_interface] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,652ms] [Warning] [omni.isaac.motion_generation.lula] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,652ms] [Warning] [omni.isaac.motion_generation.lula.kinematics] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,652ms] [Warning] [omni.isaac.motion_generation.lula.motion_policies] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,652ms] [Warning] [omni.isaac.motion_generation.lula.path_planners] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,653ms] [Warning] [omni.isaac.motion_generation.lula.trajectory_generator] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,653ms] [Warning] [omni.isaac.motion_generation.motion_policy_controller] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,653ms] [Warning] [omni.isaac.motion_generation.motion_policy_interface] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,653ms] [Warning] [omni.isaac.motion_generation.path_planner_visualizer] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,653ms] [Warning] [omni.isaac.motion_generation.path_planning_interface] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,653ms] [Warning] [omni.isaac.motion_generation.trajectory] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
2025-08-11T02:47:40Z [4,653ms] [Warning] [omni.isaac.motion_generation.world_interface] omni.isaac.motion_generation has been deprecated in favor of isaacsim.robot_motion.motion_generation. Please update your code accordingly.
[4.671s] [ext: omni.isaac.asset_browser-1.0.6] startup
2025-08-11T02:47:40Z [4,655ms] [Warning] [omni.isaac.asset_browser] omni.isaac.asset_browser has been deprecated in favor of isaacsim.asset.browser. Please update your code accordingly.
[4.674s] [ext: omni.kit.viewport.manipulator.transform-107.0.4] startup
[4.678s] [ext: omni.isaac.nucleus-1.0.6] startup
2025-08-11T02:47:40Z [4,661ms] [Warning] [omni.isaac.nucleus] omni.isaac.nucleus has been deprecated in favor of isaacsim.storage.native. Please update your code accordingly.
2025-08-11T02:47:40Z [4,661ms] [Warning] [omni.isaac.nucleus.nucleus] omni.isaac.nucleus.nucleus has been deprecated in favor of isaacsim.storage.native. Please update your code accordingly.
[4.679s] [ext: omni.fabric.commands-1.1.6] startup
[4.682s] [ext: omni.kit.manipulator.prim.core-107.0.8] startup
[4.691s] [ext: isaacsim.robot.wheeled_robots-4.0.23] startup
[4.696s] [ext: omni.isaac.utils-2.0.6] startup
[4.700s] [ext: omni.isaac.version-2.0.7] startup
2025-08-11T02:47:40Z [4,684ms] [Warning] [omni.isaac.version] omni.isaac.version has been deprecated in favor of isaacsim.core.version. Please update your code accordingly.
[4.702s] [ext: omni.isaac.wheeled_robots-3.0.7] startup
2025-08-11T02:47:40Z [4,686ms] [Warning] [omni.isaac.wheeled_robots] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,686ms] [Warning] [omni.isaac.wheeled_robots.controllers] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,687ms] [Warning] [omni.isaac.wheeled_robots.controllers.ackermann_controller] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,688ms] [Warning] [omni.isaac.wheeled_robots.controllers.differential_controller] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,689ms] [Warning] [omni.isaac.wheeled_robots.controllers.holonomic_controller] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,689ms] [Warning] [omni.isaac.wheeled_robots.controllers.quintic_path_planner] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,689ms] [Warning] [omni.isaac.wheeled_robots.controllers.stanley_control] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,689ms] [Warning] [omni.isaac.wheeled_robots.controllers.wheel_base_pose_controller] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,690ms] [Warning] [omni.isaac.wheeled_robots.impl] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,690ms] [Warning] [omni.isaac.wheeled_robots.robots] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,690ms] [Warning] [omni.isaac.wheeled_robots.robots.holonomic_robot_usd_setup] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [4,691ms] [Warning] [omni.isaac.wheeled_robots.robots.wheeled_robot] omni.isaac.wheeled_robots has been deprecated in favor of isaacsim.robot.wheeled_robots. Please update your code accordingly.
[4.709s] [ext: omni.sensors.nv.common-2.7.0-coreapi] startup
[4.722s] [ext: isaacsim.app.about-2.0.11] startup
[4.724s] [ext: omni.kit.manipulator.prim.fabric-107.0.4] startup
[4.727s] [ext: isaacsim.sensors.physics-0.3.27] startup
[4.737s] [ext: omni.kit.manipulator.prim.usd-107.0.3] startup
[4.741s] [ext: omni.sensors.net-0.4.0-coreapi] startup
[4.743s] [ext: omni.sensors.nv.materials-1.6.0-coreapi] startup
[4.750s] [ext: omni.isaac.window.about-2.0.7] startup
2025-08-11T02:47:40Z [4,735ms] [Warning] [omni.isaac.window.about] omni.isaac.window.about has been deprecated in favor of isaacsim.app.about. Please update your code accordingly.
2025-08-11T02:47:40Z [4,735ms] [Warning] [omni.isaac.window.about.about] omni.isaac.window.about.about has been deprecated in favor of isaacsim.app.about.about. Please update your code accordingly.
[4.753s] [ext: omni.kit.manipulator.camera-106.0.4] startup
[4.757s] [ext: isaacsim.robot.policy.examples-4.1.11] startup
[4.758s] [ext: omni.kit.manipulator.prim-107.0.0] startup
[4.759s] [ext: omni.kit.manipulator.selection-106.0.1] startup
[4.761s] [ext: omni.sensors.nv.ids-1.5.0-coreapi] startup
[4.764s] [ext: omni.sensors.nv.wpm-2.5.0-coreapi] startup
[4.767s] [ext: omni.sensors.nv.lidar-2.7.0-coreapi] startup
[4.777s] [ext: omni.isaac.quadruped-3.0.7] startup
2025-08-11T02:47:40Z [4,761ms] [Warning] [omni.isaac.quadruped] omni.isaac.quadruped has been deprecated in favor of isaacsim.robot.policy.examples. Please update your code accordingly.
[4.780s] [ext: omni.isaac.assets_check-0.3.13] startup
2025-08-11T02:47:40Z [4,764ms] [Warning] [omni.isaac.assets_check] omni.isaac.assets_check has been deprecated in favor of isaacsim.asset.browser. Please update your code accordingly.
[4.783s] [ext: isaacsim.gui.content_browser-0.1.9] startup
[4.793s] [ext: isaacsim.util.debug_draw-3.1.0] startup
[4.801s] [ext: omni.sensors.nv.radar-2.8.0-coreapi] startup
[4.807s] [ext: omni.isaac.franka-1.0.7] startup
2025-08-11T02:47:40Z [4,790ms] [Warning] [omni.isaac.franka] omni.isaac.franka has been deprecated in favor of isaacsim.robot.manipulators.examples.franka. Please update your code accordingly.
2025-08-11T02:47:40Z [4,790ms] [Warning] [omni.isaac.franka.franka] omni.isaac.franka has been deprecated in favor of isaacsim.robot.manipulators.examples.franka. Please update your code accordingly.
2025-08-11T02:47:40Z [4,791ms] [Warning] [omni.isaac.franka.kinematics_solver] omni.isaac.franka has been deprecated in favor of isaacsim.robot.manipulators.examples.franka. Please update your code accordingly.
[4.809s] [ext: isaacsim.gui.property-1.1.3] startup
[4.814s] [ext: isaacsim.sensors.physx-2.2.27] startup
[4.835s] [ext: isaacsim.sensors.camera-1.3.1] startup
[4.873s] [ext: isaacsim.sensors.rtx-15.5.0] startup
[4.890s] [ext: isaacsim.core.cloner-1.4.9] startup
[4.918s] [ext: omni.kit.property.isaac-1.0.6] startup
2025-08-11T02:47:40Z [4,904ms] [Warning] [omni.kit.property.isaac] omni.kit.property.isaac has been deprecated in favor of isaacsim.gui.property. Please update your code accordingly.
2025-08-11T02:47:40Z [4,909ms] [Warning] [omni.kit.property.isaac.widgets] omni.kit.property.isaac.widgets has been deprecated in favor of isaacsim.gui.property.widgets. Please update your code accordingly.
[4.927s] [ext: omni.isaac.range_sensor-4.0.6] startup
2025-08-11T02:47:40Z [4,911ms] [Warning] [omni.isaac.range_sensor] omni.isaac.range_sensor has been deprecated in favor of isaacsim.sensors.physx. Please update your code accordingly.
2025-08-11T02:47:40Z [4,911ms] [Warning] [omni.isaac.range_sensor.commands] omni.isaac.range_sensor.commands has been deprecated in favor of isaacsim.sensors.physx.commands. Please update your code accordingly.
[4.930s] [ext: omni.kit.viewport.menubar.camera-107.0.3] startup
[4.983s] [ext: omni.isaac.sensor-13.0.7] startup
2025-08-11T02:47:40Z [4,967ms] [Warning] [omni.isaac.sensor] omni.isaac.sensor has been deprecated in favor of isaacsim.sensors.camera, isaacsim.sensors.physics, isaacsim.sensors.physx, and isaacsim.sensors.rtx. Please update your code accordingly.
2025-08-11T02:47:40Z [4,967ms] [Warning] [omni.isaac.sensor.camera] omni.isaac.sensor.camera has been deprecated in favor of isaacsim.sensors.camera.camera. Please update your code accordingly.
2025-08-11T02:47:40Z [4,967ms] [Warning] [omni.isaac.sensor.camera_view] omni.isaac.sensor.camera_view has been deprecated in favor of isaacsim.sensors.camera.camera_view. Please update your code accordingly.
2025-08-11T02:47:40Z [4,967ms] [Warning] [omni.isaac.sensor.commands] omni.isaac.sensor.commands has been deprecated in favor of isaacsim.sensors.physics.commands, isaacsim.sensors.physx.commands, and isaacsim.sensors.rtx.commands. Please update your code accordingly.
2025-08-11T02:47:40Z [4,968ms] [Warning] [omni.isaac.sensor.contact_sensor] omni.isaac.sensor.contact_sensor has been deprecated in favor of isaacsim.sensors.physics.contact_sensor. Please update your code accordingly.
2025-08-11T02:47:40Z [4,968ms] [Warning] [omni.isaac.sensor.imu_sensor] omni.isaac.sensor.imu_sensor has been deprecated in favor of isaacsim.sensors.physics.imu_sensor. Please update your code accordingly.
2025-08-11T02:47:40Z [4,970ms] [Warning] [omni.isaac.sensor.lidar_rtx] omni.isaac.sensor.lidar_rtx has been deprecated in favor of isaacsim.sensors.rtx.lidar_rtx. Please update your code accordingly.
2025-08-11T02:47:40Z [4,971ms] [Warning] [omni.isaac.sensor.rotating_lidar_physX] omni.isaac.sensor.imu_sensor has been deprecated in favor of isaacsim.sensors.physix.rotating_lidar_physX. Please update your code accordingly.
2025-08-11T02:47:40Z [4,972ms] [Warning] [omni.isaac.sensor.scripts.samples.contact_sensor] omni.isaac.sensor.scripts.samples.contact_sensor has been deprecated in favor of isaacsim.sensors.physics.scripts.samples.contact_sensor. Please update your code accordingly.
2025-08-11T02:47:40Z [4,972ms] [Warning] [omni.isaac.sensor.scripts.samples.imu_sensor] omni.isaac.sensor.scripts.samples.imu_sensor has been deprecated in favor of isaacsim.sensors.physics.scripts.samples.imu_sensor. Please update your code accordingly.
2025-08-11T02:47:40Z [4,972ms] [Warning] [omni.isaac.sensor.scripts.samples.lightbeam_sensor] omni.isaac.sensor.scripts.samples.lightbeam_sensor has been deprecated in favor of isaacsim.sensors.physx.scripts.samples.lightbeam_sensor. Please update your code accordingly.
[4.991s] [ext: omni.isaac.cloner-1.0.7] startup
2025-08-11T02:47:40Z [4,974ms] [Warning] [omni.isaac.cloner] omni.isaac.cloner has been deprecated in favor of isaacsim.core.cloner. Please update your code accordingly.
[4.992s] [ext: omni.kit.viewport.menubar.render-107.0.9] startup
[5.003s] [ext: omni.kit.viewport.menubar.settings-107.0.3] startup
[5.009s] [ext: isaacsim.replicator.writers-1.0.15] startup
[5.034s] [ext: omni.kit.window.console-1.1.4] startup
[5.041s] [ext: omni.kit.window.status_bar-0.1.8] startup
[5.047s] [ext: omni.kit.viewport.actions-107.0.2] startup
[5.054s] [ext: isaacsim.replicator.domain_randomization-1.0.16] startup
[5.063s] [ext: isaacsim.replicator.examples-1.1.31] startup
[5.065s] [ext: omni.ocio-0.1.1] startup
[5.068s] [ext: omni.rtx.window.settings-0.6.19] startup
[5.085s] [ext: omni.kit.viewport.menubar.display-107.0.3] startup
[5.097s] [ext: omni.kit.window.toolbar-2.0.0] startup
[5.103s] [ext: omni.replicator.isaac-2.0.8] startup
2025-08-11T02:47:40Z [5,086ms] [Warning] [omni.replicator.isaac] omni.replicator.isaac has been deprecated in favor of isaacsim.replicator.domain_randomization, isaacsim.replicator.examples, isaacsim.replicator.writers. Please update your code accordingly.
[5.105s] [ext: omni.replicator.replicator_yaml-2.0.11] startup
[5.130s] [ext: omni.rtx.settings.core-0.6.5] startup
[5.136s] [ext: omni.usd.metrics.assembler.ui-107.3.1] startup
[5.144s] [ext: omni.isaac.surface_gripper-2.0.6] startup
2025-08-11T02:47:40Z [5,127ms] [Warning] [omni.isaac.surface_gripper] omni.isaac.surface_gripper has been deprecated in favor of isaacsim.robot.surface_gripper. Please update your code accordingly.
[5.145s] [ext: semantics.schema.editor-2.0.1] startup
[5.150s] [ext: omni.kit.ui.actions-1.0.5] startup
[5.151s] [ext: semantics.schema.property-2.0.1] startup
[5.153s] [ext: omni.isaac.universal_robots-1.0.6] startup
2025-08-11T02:47:40Z [5,136ms] [Warning] [omni.isaac.universal_robots] omni.isaac.universal_robots has been deprecated in favor of isaacsim.robot.manipulators.examples.universal_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [5,136ms] [Warning] [omni.isaac.universal_robots.kinematics_solver] omni.isaac.franka has been deprecated in favor of isaacsim.robot.manipulators.examples.universal_robots. Please update your code accordingly.
2025-08-11T02:47:40Z [5,137ms] [Warning] [omni.isaac.universal_robots.ur10] omni.isaac.franka has been deprecated in favor of isaacsim.robot.manipulators.examples.universal_robots. Please update your code accordingly.
[5.155s] [ext: omni.isaac.core-4.0.7] startup
2025-08-11T02:47:40Z [5,139ms] [Warning] [omni.isaac.core.physics_context.physics_context] omni.isaac.core.physics_context.physics_context has been deprecated in favor of isaacsim.core.api.physics_context.physics_context. Please update your code accordingly.
2025-08-11T02:47:40Z [5,139ms] [Warning] [omni.isaac.core.simulation_context.simulation_context] omni.isaac.core.simulation_context.simulation_context has been deprecated in favor of isaacsim.core.api.simulation_context.simulation_context. Please update your code accordingly.
2025-08-11T02:47:40Z [5,139ms] [Warning] [omni.isaac.core.world.world] omni.isaac.core.world.world has been deprecated in favor of isaacsim.core.api.world.world. Please update your code accordingly.
2025-08-11T02:47:40Z [5,139ms] [Warning] [omni.isaac.core] omni.isaac.core has been deprecated in favor of isaacsim.core.api. Please update your code accordingly.
[5.157s] [ext: isaacsim.gui.sensors.icon-2.0.3] startup
[5.161s] [ext: isaacsim.core.experimental.materials-0.3.0] startup
[5.164s] [ext: omni.kit.menu.common-2.0.0] startup
[5.167s] [ext: isaacsim.exp.base-5.0.0] startup
[5.167s] [ext: isaacsim.exp.base.python-5.0.0] startup
[5.168s] Simulation App Starting
2025-08-11T02:47:40Z [5,215ms] [Warning] [usdrt.population.plugin] using high frequency span is disabled
2025-08-11T02:47:40Z [5,236ms] [Warning] [omni.fabric.plugin] Warning: attribute viewportHandle not found for bucket id 9

[5.268s] app ready
Starting kit application with the following args:  ['/home/dvangelder/IsaacSim/_build/linux-x86_64/release/exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py', '/home/dvangelder/IsaacSim/_build/linux-x86_64/release/apps/isaacsim.exp.base.python.kit', '--/app/tokens/exe-path=/home/dvangelder/IsaacSim/_build/linux-x86_64/release/kit', '--/persistent/app/viewport/displayOptions=3094', '--/rtx/materialDb/syncLoads=True', '--/rtx/hydra/materialSyncLoads=True', '--/omni.kit.plugin/syncUsdLoads=True', '--/app/renderer/resolution/width=1280', '--/app/renderer/resolution/height=720', '--/app/window/width=1440', '--/app/window/height=900', '--/renderer/multiGpu/enabled=True', '--/app/fastShutdown=True', '--/app/installSignalHandlers=0', '--ext-folder', '/home/dvangelder/IsaacSim/_build/linux-x86_64/release/exts', '--ext-folder', '/home/dvangelder/IsaacSim/_build/linux-x86_64/release/apps', '--/physics/cudaDevice=0', '--/renderer/multiGpu/maxGpuCount=1', '--/plugins/carb.tasking.plugin/threadCount=32', '--/plugins/omni.tbb.globalcontrol/maxThreadCount=32', '--portable', '--no-window', '--/app/window/hideUi=1']
Passing the following args to the base kit application:  ['--num-cameras', '50', '--resolution', '1920', '1080', '--num-gpus', '1', '--/crashreporter/enabled=false']
Warp 1.7.1 initialized:
   CUDA Toolkit 12.8, Driver 12.8
   Devices:
     "cpu"      : "x86_64"
     "cuda:0"   : "NVIDIA GeForce RTX 5090" (31 GiB, sm_120, mempool enabled)
   Kernel cache:
     /home/dvangelder/.cache/warp/1.7.1
2025-08-11 02:47:42 [7,443ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Generating formatted report = True
2025-08-11 02:47:42 [7,444ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Using metrics backend = OmniPerfKPIFile
2025-08-11 02:47:42 [7,444ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Local folder location = /tmp
2025-08-11 02:47:42 [7,444ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Starting
2025-08-11 02:47:42 [7,444ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Test mode = False
2025-08-11 02:47:42 [7,444ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Starting phase: loading
2025-08-11 02:47:57 [22,418ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Starting phase: benchmark
2025-08-11T02:47:40Z [5,339ms] [Warning] [carb] Acquiring non optional plugin interface which is not listed as dependency: [carb::dictionary::ISerializer v1.1] (plugin: carb.dictionary.serializer-json.plugin), by client: carb.scenerenderer-rtx.plugin. Add it to CARB_PLUGIN_IMPL_DEPS() macro of a client.
2025-08-11T02:47:42Z [6,985ms] [Warning] [usdrt.hydra.fabric_scene_delegate.plugin] using high frequency span is disabled
2025-08-11T02:47:42Z [6,985ms] [Warning] [usdrt.hydra.fabric_scene_delegate.plugin] using high frequency span is disabled
2025-08-11T02:47:42Z [6,986ms] [Warning] [usdrt.hydra.fabric_scene_delegate.plugin] using high frequency span with attrs is disabled
2025-08-11T02:47:42Z [7,004ms] [Warning] [usdrt.hydra.fabric_scene_delegate.plugin] using high frequency span is disabled
2025-08-11T02:47:42Z [7,004ms] [Warning] [usdrt.hydra.fabric_scene_delegate.plugin] using high frequency span is disabled
[7.240s] Viewport updates disabled
[7.240s] Simulation App Startup Complete
[7.249s] [ext: isaacsim.benchmark.services-3.0.5] startup
2025-08-11T02:47:44Z [9,195ms] [Warning] [usdrt.hydra.fabric_scene_delegate.plugin] using high frequency span with attrs is disabled
2025-08-11T02:47:45Z [9,540ms] [Warning] [omni.fabric.plugin] Warning: attribute outputs:prims not found for path /Replicator/SDGPipeline/OgnGroup

2025-08-11T02:47:46Z [11,125ms] [Warning] [carb] Client omni.hydratexture.plugin has acquired [carb::settings::ISettings v1.0] 100 times. Consider accessing this interface with carb::getCachedInterface() (Performance warning)
2025-08-11T02:47:46Z [11,138ms] [Warning] [carb] Client omni.hydratexture.plugin has acquired [carb::dictionary::IDictionary v1.1] 100 times. Consider accessing this interface with carb::getCachedInterface() (Performance warning)
2025-08-11T02:47:47Z [11,783ms] [Warning] [carb] Client omni.kit.viewport.legacy_gizmos has acquired [carb::settings::ISettings v1.0] 100 times. Consider accessing this interface with carb::getCachedInterface() (Performance warning)
2025-08-11T02:47:47Z [11,783ms] [Warning] [carb] Client omni.kit.viewport.legacy_gizmos has acquired [carb::dictionary::IDictionary v1.1] 100 times. Consider accessing this interface with carb::getCachedInterface() (Performance warning)
2025-08-11T02:47:59Z [23,519ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,520ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,521ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,522ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,523ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,524ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,526ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,527ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,528ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,529ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,531ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,532ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,533ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,534ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,535ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,536ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,537ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,538ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,539ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,541ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,542ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,543ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,544ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,545ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,546ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,547ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,548ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,549ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,550ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,552ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,553ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,554ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,555ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,556ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,557ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,558ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,559ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,560ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,561ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,562ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,563ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,565ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,566ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,567ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,568ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,569ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,570ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11 02:52:25 [289,968ms] [WARNING] [isaacsim.benchmark.services.datarecorders.frametime] Unable to calculate frametime stats: mean requires at least one data point
2025-08-11 02:52:25 [289,969ms] [WARNING] [isaacsim.benchmark.services.datarecorders.frametime] Unable to calculate frametime stats: mean requires at least one data point
2025-08-11 02:52:25 [289,969ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Stopping
2025-08-11 02:52:25 [289,969ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Writing metrics data.
2025-08-11 02:52:25 [289,969ms] [INFO] [isaacsim.benchmark.services.base_isaac_benchmark] Metrics type = OmniPerfKPIFile
2025-08-11 02:52:25 [289,969ms] [INFO] [isaacsim.benchmark.services.metrics.backend] 
loading Metrics:
workflow_name: benchmark_camera
num_cameras: 50
width: 1920
height: 1080
num_gpus: 1
phase: loading
System Memory RSS: 6.015 GB
System Memory VMS: 154.825 GB
System Memory USS: 5.961 GB
GPU Memory Tracked: 9.761 GB
GPU Memory Dedicated: 21.234 GB
System CPU iowait: 0.0 %
System CPU system: 3.0 %
System CPU user: 11.0 %
System CPU idle: 86.0 %
num_cpus: 32 
gpu_device_name: NVIDIA GeForce RTX 5090 
Runtime: 14893.203 ms
2025-08-11 02:52:25 [289,969ms] [INFO] [isaacsim.benchmark.services.metrics.backend] 
benchmark Metrics:
workflow_name: benchmark_camera
num_cameras: 50
width: 1920
height: 1080
num_gpus: 1
phase: benchmark
System Memory RSS: 6.516 GB
System Memory VMS: 157.952 GB
System Memory USS: 6.461 GB
GPU Memory Tracked: 17.089 GB
GPU Memory Dedicated: 27.287 GB
System CPU iowait: 0.0 %
System CPU system: 2.0 %
System CPU user: 4.0 %
System CPU idle: 94.0 %
num_cpus: 32 
gpu_device_name: NVIDIA GeForce RTX 5090 
Mean App_Update Frametime: 445.47 ms
Stdev App_Update Frametime: 1.84 ms
Min App_Update Frametime: 442.3 ms
Max App_Update Frametime: 449.23 ms
Mean Physics Frametime: 0.5 ms
Stdev Physics Frametime: 0.02 ms
Min Physics Frametime: 0.45 ms
Max Physics Frametime: 0.54 ms
Mean FPS: 2.245 FPS
Real Time Factor: 0.037 
Runtime: 267473.422 ms
2025-08-11 02:52:25 [289,970ms] [INFO] [isaacsim.benchmark.services.metrics.backend] Writing metrics to /tmp/kpis_benchmark_camera.json
2025-08-11T02:47:59Z [23,571ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:47:59Z [23,572ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:48:13Z [37,640ms] [Warning] [carb] Client gpu.foundation.plugin has acquired [gpu::unstable::IMemoryBudgetManagerFactory v0.1] 100 times. Consider accessing this interface with carb::getCachedInterface() (Performance warning)
2025-08-11T02:52:25Z [290,087ms] [Warning] [carb] Client omni.kit.viewport.legacy_gizmos has acquired [carb::input::IInput v1.2] 100 times. Consider accessing this interface with carb::getCachedInterface() (Performance warning)
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,405ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,406ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,407ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
2025-08-11T02:52:27Z [292,408ms] [Warning] [carb] Plugin interface for a client: omni.hydratexture.plugin was already released.
[292.614s] Simulation App Shutting Down
