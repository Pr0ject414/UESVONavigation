#include "SVONavDataRenderingComponent.h"

#include "SVOHelpers.h"
#include "SVONavigationData.h"
#include "Core/SVONavigationDataChunkActor.h"

#include "Debug/DebugDrawService.h"
#include "Engine/CollisionProfile.h"

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
#include "Engine/Canvas.h"
#endif

#if WITH_EDITOR
#include <Editor.h>
#include <EditorViewportClient.h>
#endif

static const FColor OccludedVoxelColor = FColor::Orange;
static const FColor FreeVoxelColor = FColor::Green;
static const FColor PortalColor = FColor::Cyan;

FSVONavigationMeshSceneProxy::FSVONavigationMeshSceneProxy( const UPrimitiveComponent * component ) :
    FDebugRenderSceneProxy( component )
{
    DrawType = EDrawType::WireMesh;

    RenderingComponent = MakeWeakObjectPtr( const_cast< USVONavDataRenderingComponent * >( Cast< USVONavDataRenderingComponent >( component ) ) );
    NavigationData = MakeWeakObjectPtr( Cast< ASVONavigationData >( RenderingComponent->GetOwner() ) );

    if ( NavigationData == nullptr )
    {
        return;
    }

    const auto & debug_infos = NavigationData->GetDebugInfos();

    // Helper lambda to draw a single volume (reused for Legacy and Chunk data)
    auto ProcessVolumeData = [&](const FSVOVolumeNavigationData& Data)
    {
        const auto & octree_data = Data.GetData();
        const auto layer_count = octree_data.GetLayerCount();

        if ( layer_count == 0 )
        {
            return;
        }

        if ( debug_infos.bDebugDrawBounds )
        {
            Boxes.Emplace( Data.GetData().GetNavigationBounds(), FColor::White );
        }

        // -- Draw Portals --
        if (debug_infos.bDebugDrawPortals)
        {
            const TArray<FSVOPortal>& Portals = Data.GetPortals();
            for (const FSVOPortal& Portal : Portals)
            {
                // Draw portal as a wireframe box
                Boxes.Emplace(FBox::BuildAABB(Portal.Location, Portal.Extent), PortalColor);
            }
        }

        const auto & leaf_nodes = octree_data.GetLeafNodes();
        const auto leaf_node_extent = leaf_nodes.GetLeafNodeExtent();

        FSVONodeAddress node_address_for_neighbors;
        if ( !debug_infos.NeighborLinksForNodeAddress.IsEmpty() )
        {
            TArray< FString > parts;
            debug_infos.NeighborLinksForNodeAddress.ParseIntoArrayWS( parts, TEXT( " " ) );

            if ( parts.Num() == 3 )
            {
                node_address_for_neighbors.LayerIndex = FCString::Atoi( *parts[ 0 ] );
                node_address_for_neighbors.NodeIndex = FCString::Atoi( *parts[ 1 ] );
                node_address_for_neighbors.SubNodeIndex = FCString::Atoi( *parts[ 2 ] );
            }
        }

        if ( debug_infos.bDebugDrawNeighborLinks && node_address_for_neighbors.IsValid() )
        {
            DrawNeighborInfos( Data, node_address_for_neighbors );
            return;
        }

        if ( debug_infos.bDebugDrawLayers )
        {
            const auto corrected_layer_index = FMath::Clamp( static_cast< int >( debug_infos.LayerIndexToDraw ), 0, layer_count - 1 );
            const auto node_extent = Data.GetData().GetLayer( corrected_layer_index ).GetNodeExtent();

            for ( const auto & node : octree_data.GetLayer( corrected_layer_index ).GetNodes() )
            {
                const auto code = node.MortonCode;

                if ( corrected_layer_index == 0 )
                {
                    const auto leaf_node_position = Data.GetLeafNodePositionFromMortonCode( code );

                    if ( AddVoxelToBoxes( leaf_node_position, leaf_node_extent, node.HasChildren() ) )
                    {
                        AddNodeTextInfos( code, 0, leaf_node_position );
                    }
                }
                else
                {
                    const auto position = Data.GetNodePositionFromLayerAndMortonCode( corrected_layer_index, code );

                    if ( AddVoxelToBoxes( position, node_extent, node.HasChildren() ) )
                    {
                        AddNodeTextInfos( code, corrected_layer_index, position );
                    }
                }
            }
        }

        if ( debug_infos.bDebugDrawSubNodes )
        {
            const auto & leaf_layer = octree_data.GetLayer( 0 );
            const auto leaf_sub_node_size = leaf_nodes.GetLeafSubNodeSize();
            const auto leaf_sub_node_extent = leaf_nodes.GetLeafSubNodeExtent();

            for ( const auto & leaf_node : leaf_layer.GetNodes() )
            {
                if ( leaf_node.HasChildren() )
                {
                    const auto & leaf = octree_data.GetLeafNodes().GetLeafNode( leaf_node.FirstChild.NodeIndex );
                    const auto code = leaf_node.MortonCode;
                    const auto leaf_node_position = Data.GetLeafNodePositionFromMortonCode( code );

                    for ( SubNodeIndex sub_node_index = 0; sub_node_index < 64; sub_node_index++ )
                    {
                        const auto sub_node_morton_coords = FSVOHelpers::GetVectorFromMortonCode( sub_node_index );
                        const auto sub_node_location = leaf_node_position - leaf_node_extent + sub_node_morton_coords * leaf_sub_node_size + leaf_sub_node_extent;
                        const bool is_sub_node_occluded = leaf.IsSubNodeOccluded( sub_node_index );

                        if ( AddVoxelToBoxes( sub_node_location, leaf_sub_node_extent, is_sub_node_occluded ) )
                        {
                            AddNodeTextInfos( sub_node_index, 0, sub_node_location );
                        }
                    }
                }
            }
        }
    };

    // 1. Process Legacy Monolithic Data
    const auto & all_navigation_bounds_data = NavigationData->GetVolumeNavigationData();
    for ( const auto & navigation_bounds_data : all_navigation_bounds_data )
    {
        ProcessVolumeData(navigation_bounds_data);
    }

    // 2. Process World Partition Chunk Actors
    const TArray<TObjectPtr<ASVONavigationDataChunkActor>>& ChunkActors = NavigationData->GetChunkActors();
    for (const auto& ChunkActor : ChunkActors)
    {
        if (ChunkActor)
        {
            ProcessVolumeData(ChunkActor->GetVolumeNavigationData());
        }
    }
}

FSVONavigationMeshSceneProxy::~FSVONavigationMeshSceneProxy()
{
}

SIZE_T FSVONavigationMeshSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

bool FSVONavigationMeshSceneProxy::AddVoxelToBoxes( const FVector & voxel_location, const float node_extent, const bool is_occluded )
{
    const auto & debug_infos = NavigationData->GetDebugInfos();

    if ( debug_infos.bDebugDrawFreeVoxels && !is_occluded )
    {
        Boxes.Emplace( FBox::BuildAABB( voxel_location, FVector( node_extent ) ), FreeVoxelColor );

        return true;
    }
    if ( debug_infos.bDebugDrawOccludedVoxels && is_occluded )
    {
        Boxes.Emplace( FBox::BuildAABB( voxel_location, FVector( node_extent ) ), OccludedVoxelColor );

        return true;
    }

    return false;
}

void FSVONavigationMeshSceneProxy::AddNodeTextInfos( const MortonCode node_morton_code, const LayerIndex node_layer_index, const FVector & node_position )
{
    const auto & debug_infos = NavigationData->GetDebugInfos();

    static constexpr float vertical_offset_increment = 40.0f;

    auto vertical_offset = 0.0f;
    if ( debug_infos.bDebugDrawMortonCoords )
    {
        Texts.Emplace( FString::Printf( TEXT( "%i:%llu" ), node_layer_index, node_morton_code ), node_position, FLinearColor::Black );
        vertical_offset += vertical_offset_increment;
    }
    if ( debug_infos.bDebugDrawNodeCoords )
    {
        const FIntVector morton_coords = FIntVector( FSVOHelpers::GetVectorFromMortonCode( node_morton_code ) );
        Texts.Emplace( FString::Printf( TEXT( "%s" ), *morton_coords.ToString() ), node_position + FVector( 0.0f, 0.0f, vertical_offset ), FLinearColor::Black );
        vertical_offset += vertical_offset_increment;
    }
    if ( debug_infos.bDebugDrawNodeAddresses )
    {
        const FSVONodeAddress node_address( node_layer_index, node_morton_code );

        Texts.Emplace( FString::Printf( TEXT( "%s" ), *node_address.ToString() ), node_position + FVector( 0.0f, 0.0f, vertical_offset ), FLinearColor::Black );
        vertical_offset += vertical_offset_increment;
    }
    if ( debug_infos.bDebugDrawNodeLocation )
    {
        Texts.Emplace( FString::Printf( TEXT( "%s" ), *node_position.ToCompactString() ), node_position + FVector( 0.0f, 0.0f, vertical_offset ), FLinearColor::Black );
    }
}

void FSVONavigationMeshSceneProxy::DrawNeighborInfos( const FSVOVolumeNavigationData & navigation_bounds_data, const FSVONodeAddress & node_address )
{
    const auto & octree_data = navigation_bounds_data.GetData();

    TArray< FSVONodeAddress > neighbor_node_addresses;
    navigation_bounds_data.GetNodeNeighbors( neighbor_node_addresses, node_address );

    const auto node_position = navigation_bounds_data.GetNodePositionFromAddress( node_address, false );
    const auto node_extent = navigation_bounds_data.GetData().GetLayer( node_address.LayerIndex ).GetNodeExtent();

    AddVoxelToBoxes( node_position, node_extent, false );
    Spheres.Emplace( node_extent * 0.5f, node_position, FColor::Black );

    const auto & leaf_nodes = octree_data.GetLeafNodes();
    const auto leaf_node_extent = leaf_nodes.GetLeafNodeExtent();
    const auto leaf_sub_node_size = leaf_nodes.GetLeafSubNodeSize();
    const auto leaf_sub_node_extent = leaf_nodes.GetLeafSubNodeExtent();

    for ( const auto & neighbor_node_address : neighbor_node_addresses )
    {
        FColor color;

        if ( node_address.LayerIndex == neighbor_node_address.LayerIndex )
        {
            color = FColor::Purple;
        }
        else if ( node_address.LayerIndex > neighbor_node_address.LayerIndex )
        {
            color = FColor::Cyan;
        }
        else
        {
            color = FColor::Red;
        }

        Lines.Emplace( node_position, navigation_bounds_data.GetNodePositionFromAddress( neighbor_node_address, true ), color, 5.0f );

        FVector neighbor_node_position;
        float neighbor_node_extent;

        neighbor_node_position = navigation_bounds_data.GetNodePositionFromAddress( neighbor_node_address, true );

        if ( neighbor_node_address.SubNodeIndex > 0 )
        {
            neighbor_node_extent = leaf_nodes.GetLeafSubNodeExtent();
        }
        else
        {
            neighbor_node_extent = navigation_bounds_data.GetData().GetLayer( neighbor_node_address.LayerIndex ).GetNodeExtent();
        }

        Boxes.Emplace( FBox::BuildAABB( neighbor_node_position, FVector( neighbor_node_extent ) ), color );
        Texts.Emplace( FString::Printf( TEXT( "%s" ), *neighbor_node_address.ToString() ), neighbor_node_position, FLinearColor::Black );
        Spheres.Emplace( neighbor_node_extent * 0.5f, neighbor_node_position, color );
    }
}

FPrimitiveViewRelevance FSVONavigationMeshSceneProxy::GetViewRelevance( const FSceneView * view ) const
{
    const bool bVisible = !!view->Family->EngineShowFlags.Navigation;
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = bVisible && IsShown( view );
    Result.bDynamicRelevance = true;
    // ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
    Result.bSeparateTranslucency = Result.bNormalTranslucency = bVisible && IsShown( view );
    return Result;
}

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST

void FSVODebugDrawDelegateHelper::InitDelegateHelper( const FSVONavigationMeshSceneProxy * scene_proxy )
{
    Super::InitDelegateHelper( scene_proxy );

    NavigationData = scene_proxy->NavigationData;
}

void FSVODebugDrawDelegateHelper::RegisterDebugDrawDelegateInternal()
{
    if ( State == InitializedState )
    {
        DebugTextDrawingDelegate = FDebugDrawDelegate::CreateRaw( this, &FSVODebugDrawDelegateHelper::DrawDebugLabels );
        DebugTextDrawingDelegateHandle = UDebugDrawService::Register( TEXT( "Navigation" ), DebugTextDrawingDelegate );
        State = RegisteredState;
    }
}

void FSVODebugDrawDelegateHelper::UnregisterDebugDrawDelegate()
{
    if ( State == RegisteredState )
    {
        check( DebugTextDrawingDelegate.IsBound() );
        UDebugDrawService::Unregister( DebugTextDrawingDelegateHandle );
        State = InitializedState;
    }
}
#endif

USVONavDataRenderingComponent::USVONavDataRenderingComponent()
{
    SetCollisionProfileName( UCollisionProfile::NoCollision_ProfileName );

    bIsEditorOnly = true;
    bSelectable = false;
    bForcesUpdate = false;
}

FPrimitiveSceneProxy * USVONavDataRenderingComponent::CreateSceneProxy()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    FSVONavigationMeshSceneProxy * proxy = nullptr;

    if ( IsVisible() )
    {
        if ( const ASVONavigationData * navigation_data = Cast< ASVONavigationData >( GetOwner() ) )
        {
            // Removed IsDrawingEnabled() check to ensure it draws if the component is visible and show flags are on
            proxy = new FSVONavigationMeshSceneProxy( this );
        }
    }

    if ( proxy != nullptr )
    {
        DebugDrawDelegateManager.InitDelegateHelper( proxy );
        DebugDrawDelegateManager.ReregisterDebugDrawDelegate();
    }

    return proxy;
#else
    return nullptr;
#endif
}

FBoxSphereBounds USVONavDataRenderingComponent::CalcBounds( const FTransform & LocalToWorld ) const
{
    FBox bounding_box( ForceInit );

    if ( const ASVONavigationData * navigation_data = Cast< ASVONavigationData >( GetOwner() ) )
    {
        bounding_box = navigation_data->GetBoundingBox();
    }

    return FBoxSphereBounds( bounding_box );
}

void USVONavDataRenderingComponent::CreateRenderState_Concurrent( FRegisterComponentContext * context )
{
    Super::CreateRenderState_Concurrent( context );

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.RequestRegisterDebugDrawDelegate( context );
#endif
}

void USVONavDataRenderingComponent::DestroyRenderState_Concurrent()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.UnregisterDebugDrawDelegate();
#endif

    Super::DestroyRenderState_Concurrent();
}

bool USVONavDataRenderingComponent::IsNavigationShowFlagSet( const UWorld * world )
{
    bool show_navigation = false;

    FWorldContext * world_context = GEngine->GetWorldContextFromWorld( world );

#if WITH_EDITOR
    if ( GEditor != nullptr && world_context && world_context->WorldType != EWorldType::Game )
    {
        show_navigation = world_context->GameViewport != nullptr && world_context->GameViewport->EngineShowFlags.Navigation;
        if ( show_navigation == false )
        {
            // we have to check all viewports because we can't distinguish between SIE and PIE at this point.
            for ( FEditorViewportClient * current_viewport : GEditor->GetAllViewportClients() )
            {
                if ( current_viewport && current_viewport->EngineShowFlags.Navigation )
                {
                    show_navigation = true;
                    break;
                }
            }
        }
    }
    else
#endif // WITH_EDITOR
    {
        show_navigation = world_context && world_context->GameViewport && world_context->GameViewport->EngineShowFlags.Navigation;
    }

    return show_navigation;
}