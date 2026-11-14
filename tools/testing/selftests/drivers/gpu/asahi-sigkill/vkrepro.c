// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <vulkan/vulkan.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define GROUPS 65536u
#define CHECK(call) do { \
    VkResult result = (call); \
    if (result != VK_SUCCESS) { \
        fprintf(stderr, "%s failed: %d\n", #call, result); \
        return 1; \
    } \
} while (0)

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s shader.spv\n", argv[0]);
        return 1;
    }
    VkApplicationInfo app = {
        .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
        .apiVersion = VK_API_VERSION_1_1,
    };
    VkInstanceCreateInfo instance_info = {
        .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
        .pApplicationInfo = &app,
    };
    VkInstance instance;
    CHECK(vkCreateInstance(&instance_info, NULL, &instance));

    uint32_t device_count = 0;
    CHECK(vkEnumeratePhysicalDevices(instance, &device_count, NULL));
    if (!device_count) {
        fprintf(stderr, "no Vulkan device\n");
        return 1;
    }
    VkPhysicalDevice *devices = calloc(device_count, sizeof(*devices));
    if (!devices)
        return 1;
    CHECK(vkEnumeratePhysicalDevices(instance, &device_count, devices));
    VkPhysicalDevice physical = VK_NULL_HANDLE;
    for (uint32_t i = 0; i < device_count; i++) {
        VkPhysicalDeviceProperties properties;
        vkGetPhysicalDeviceProperties(devices[i], &properties);
        if (strstr(properties.deviceName, "Apple")) {
            physical = devices[i];
            printf("device %s\n", properties.deviceName);
            break;
        }
    }
    free(devices);
    if (physical == VK_NULL_HANDLE) {
        fprintf(stderr, "no Apple Vulkan device\n");
        return 1;
    }

    uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &family_count, NULL);
    VkQueueFamilyProperties *families = calloc(family_count, sizeof(*families));
    if (!families)
        return 1;
    vkGetPhysicalDeviceQueueFamilyProperties(physical, &family_count, families);
    uint32_t family = family_count;
    for (uint32_t i = 0; i < family_count; i++) {
        if (families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
            family = i;
            break;
        }
    }
    free(families);
    if (family == family_count) {
        fprintf(stderr, "no Vulkan compute queue\n");
        return 1;
    }

    float priority = 1.0f;
    VkDeviceQueueCreateInfo queue_info = {
        .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
        .queueFamilyIndex = family,
        .queueCount = 1,
        .pQueuePriorities = &priority,
    };
    VkDeviceCreateInfo device_info = {
        .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
        .queueCreateInfoCount = 1,
        .pQueueCreateInfos = &queue_info,
    };
    VkDevice device;
    CHECK(vkCreateDevice(physical, &device_info, NULL, &device));
    VkQueue queue;
    vkGetDeviceQueue(device, family, 0, &queue);

    VkBufferCreateInfo buffer_info = {
        .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
        .size = (VkDeviceSize)GROUPS * 256 * sizeof(float),
        .usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
        .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
    };
    VkBuffer buffer;
    CHECK(vkCreateBuffer(device, &buffer_info, NULL, &buffer));
    VkMemoryRequirements requirements;
    vkGetBufferMemoryRequirements(device, buffer, &requirements);
    VkPhysicalDeviceMemoryProperties memory_properties;
    vkGetPhysicalDeviceMemoryProperties(physical, &memory_properties);
    uint32_t memory_type = memory_properties.memoryTypeCount;
    for (uint32_t i = 0; i < memory_properties.memoryTypeCount; i++) {
        if ((requirements.memoryTypeBits & (1u << i)) &&
            (memory_properties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT)) {
            memory_type = i;
            break;
        }
    }
    if (memory_type == memory_properties.memoryTypeCount) {
        fprintf(stderr, "no host-visible memory type\n");
        return 1;
    }
    VkMemoryAllocateInfo allocation = {
        .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
        .allocationSize = requirements.size,
        .memoryTypeIndex = memory_type,
    };
    VkDeviceMemory memory;
    CHECK(vkAllocateMemory(device, &allocation, NULL, &memory));
    CHECK(vkBindBufferMemory(device, buffer, memory, 0));

    FILE *shader_file = fopen(argv[1], "rb");
    if (!shader_file) {
        perror("shader");
        return 1;
    }
    if (fseek(shader_file, 0, SEEK_END))
        return 1;
    long shader_size = ftell(shader_file);
    if (shader_size <= 0 || fseek(shader_file, 0, SEEK_SET))
        return 1;
    uint32_t *shader_code = malloc((size_t)shader_size);
    if (!shader_code)
        return 1;
    if (fread(shader_code, 1, (size_t)shader_size, shader_file) != (size_t)shader_size)
        return 1;
    fclose(shader_file);
    VkShaderModuleCreateInfo shader_info = {
        .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
        .codeSize = (size_t)shader_size,
        .pCode = shader_code,
    };
    VkShaderModule shader;
    CHECK(vkCreateShaderModule(device, &shader_info, NULL, &shader));
    free(shader_code);

    VkDescriptorSetLayoutBinding binding = {
        .binding = 0,
        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        .descriptorCount = 1,
        .stageFlags = VK_SHADER_STAGE_COMPUTE_BIT,
    };
    VkDescriptorSetLayoutCreateInfo set_layout_info = {
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
        .bindingCount = 1,
        .pBindings = &binding,
    };
    VkDescriptorSetLayout set_layout;
    CHECK(vkCreateDescriptorSetLayout(device, &set_layout_info, NULL, &set_layout));
    VkPipelineLayoutCreateInfo pipeline_layout_info = {
        .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
        .setLayoutCount = 1,
        .pSetLayouts = &set_layout,
    };
    VkPipelineLayout pipeline_layout;
    CHECK(vkCreatePipelineLayout(device, &pipeline_layout_info, NULL, &pipeline_layout));
    VkComputePipelineCreateInfo pipeline_info = {
        .sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO,
        .layout = pipeline_layout,
        .stage = {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
            .stage = VK_SHADER_STAGE_COMPUTE_BIT,
            .module = shader,
            .pName = "main",
        },
    };
    VkPipeline pipeline;
    CHECK(vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipeline_info, NULL, &pipeline));

    VkDescriptorPoolSize pool_size = {
        .type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        .descriptorCount = 1,
    };
    VkDescriptorPoolCreateInfo pool_info = {
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
        .maxSets = 1,
        .poolSizeCount = 1,
        .pPoolSizes = &pool_size,
    };
    VkDescriptorPool pool;
    CHECK(vkCreateDescriptorPool(device, &pool_info, NULL, &pool));
    VkDescriptorSetAllocateInfo set_info = {
        .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
        .descriptorPool = pool,
        .descriptorSetCount = 1,
        .pSetLayouts = &set_layout,
    };
    VkDescriptorSet set;
    CHECK(vkAllocateDescriptorSets(device, &set_info, &set));
    VkDescriptorBufferInfo descriptor_buffer = {
        .buffer = buffer,
        .range = VK_WHOLE_SIZE,
    };
    VkWriteDescriptorSet write = {
        .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
        .dstSet = set,
        .descriptorCount = 1,
        .descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
        .pBufferInfo = &descriptor_buffer,
    };
    vkUpdateDescriptorSets(device, 1, &write, 0, NULL);

    VkCommandPoolCreateInfo command_pool_info = {
        .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
        .queueFamilyIndex = family,
    };
    VkCommandPool command_pool;
    CHECK(vkCreateCommandPool(device, &command_pool_info, NULL, &command_pool));
    VkCommandBufferAllocateInfo command_info = {
        .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
        .commandPool = command_pool,
        .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
        .commandBufferCount = 2,
    };
    VkCommandBuffer commands[2];
    CHECK(vkAllocateCommandBuffers(device, &command_info, commands));
    VkFence fences[2];
    VkFenceCreateInfo fence_info = { .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    for (int i = 0; i < 2; i++) {
        VkCommandBufferBeginInfo begin = { .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
        CHECK(vkBeginCommandBuffer(commands[i], &begin));
        vkCmdBindPipeline(commands[i], VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
        vkCmdBindDescriptorSets(commands[i], VK_PIPELINE_BIND_POINT_COMPUTE,
                                pipeline_layout, 0, 1, &set, 0, NULL);
        vkCmdDispatch(commands[i], GROUPS, 1, 1);
        CHECK(vkEndCommandBuffer(commands[i]));
        CHECK(vkCreateFence(device, &fence_info, NULL, &fences[i]));
    }

    VkSubmitInfo submits[2] = {0};
    for (int i = 0; i < 2; i++) {
        submits[i].sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submits[i].commandBufferCount = 1;
        submits[i].pCommandBuffers = &commands[i];
    }
    CHECK(vkQueueSubmit(queue, 1, &submits[0], fences[0]));
    CHECK(vkQueueSubmit(queue, 1, &submits[1], fences[1]));
    puts("ready");
    fflush(stdout);

    int current = 0;
    for (;;) {
        CHECK(vkWaitForFences(device, 1, &fences[current], VK_TRUE, UINT64_MAX));
        CHECK(vkResetFences(device, 1, &fences[current]));
        CHECK(vkQueueSubmit(queue, 1, &submits[current], fences[current]));
        current ^= 1;
    }
}
