# TAK Ports, Protocols & Procedures - Quick Reference

## TAK Server Standard Ports

| Port | Protocol | Purpose | Security | Notes |
|------|----------|---------|----------|-------|
| **8089** | TCP/TLS | Primary TAK Server Connection | Encrypted | Default for ATAK/WinTAK clients |
| **8087** | TCP | Backup TAK Server Connection | Unencrypted | Fallback/legacy support |
| **8443** | HTTPS | TAK Server Web Interface | Encrypted | Administration portal |
| **8446** | HTTPS | WebTAK Interface | Encrypted | Browser-based TAK client |
| **9000** | TCP | TAK Server Federation | Varies | Server-to-server communication |
| **6969** | UDP | Mesh SA Multicast | Unencrypted | Peer-to-peer discovery |
| **4900** | TCP | Firmware/Software Updates | Varies | Over-the-air updates |

## TAK Multicast Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Multicast Group** | 239.2.3.1 | TAK Mesh SA multicast address |
| **Port** | 6969 | Standard TAK mesh port |
| **Protocol** | UDP | User Datagram Protocol |
| **TTL** | 1-32 | Time-to-live (network hops) |
| **Interface** | Auto/Specific | Network interface binding |

## TAK Protocol Specifications

### TAK Protocol Version 0 (XML)
```xml
<event version="2.0" uid="unique-id" type="a-f-G-E" how="m-g" 
       time="2023-07-18T15:25:09.00Z" start="2023-07-18T15:25:09.00Z" 
       stale="2023-07-18T20:25:09.00Z">
  <point lat="40.7128" lon="-74.0060" hae="10.0" ce="5.0" le="10.0"/>
  <detail>
    <contact callsign="CALLSIGN" endpoint="*:-1:stcp"/>
    <__group name="Cyan" role="Team Member"/>
  </detail>
</event>
```

### TAK Protocol Version 1 (Protobuf)
- **Mesh Format**: `[191][1][191][protobuf-payload]`
- **Stream Format**: `[191][varint-length][protobuf-payload]`
- **Compression**: Optional GZIP compression
- **Encryption**: TLS transport layer security

## Certificate Configuration

### X.509 Certificate Requirements
- **Key Size**: RSA 2048+ or ECDSA P-256+
- **Hash Algorithm**: SHA-256 or better
- **Format**: PEM or PKCS#12 (.p12)
- **Validity**: Maximum 2 years recommended
- **Extensions**: Key Usage, Extended Key Usage

### Certificate Authority Setup
```bash
# Generate CA private key
openssl genrsa -out ca-key.pem 2048

# Create CA certificate
openssl req -new -x509 -days 365 -key ca-key.pem -out ca.pem

# Generate client certificate request
openssl req -new -key client-key.pem -out client.csr

# Sign client certificate
openssl x509 -req -in client.csr -CA ca.pem -CAkey ca-key.pem -out client.pem
```

## CoT Message Types Reference

### Entity Classification
| CoT Type | Classification | Description |
|----------|----------------|-------------|
| **a-f-A-*** | Friendly Air | Aircraft, drones, helicopters |
| **a-f-G-*** | Friendly Ground | Vehicles, personnel, equipment |
| **a-f-S-*** | Friendly Sea | Ships, submarines, boats |
| **a-h-*** | Hostile | Enemy forces and equipment |
| **a-n-*** | Neutral | Civilian or unaligned entities |
| **a-u-*** | Unknown | Unidentified contacts |
| **a-p-*** | Pending | Entities awaiting classification |

### Special Message Types
| CoT Type | Purpose | Description |
|----------|---------|-------------|
| **b-t-f** | Chat | Text messaging between users |
| **b-a-*** | Alerts | Emergency notifications |
| **t-x-c-t** | Ping | Server connectivity test |
| **t-x-m-*** | Mission | Mission management messages |
| **u-d-*** | Drawing | User-drawn shapes and annotations |
| **b-m-p-*** | Points | Waypoints, routes, markers |

## Network Configuration Procedures

### Firewall Rules (Linux iptables)
```bash
# Allow TAK Server ports
iptables -A INPUT -p tcp --dport 8089 -j ACCEPT  # TLS
iptables -A INPUT -p tcp --dport 8087 -j ACCEPT  # TCP
iptables -A INPUT -p tcp --dport 8443 -j ACCEPT  # HTTPS
iptables -A INPUT -p tcp --dport 8446 -j ACCEPT  # WebTAK

# Allow TAK multicast
iptables -A INPUT -p udp --dport 6969 -j ACCEPT
iptables -A INPUT -d 239.2.3.1 -j ACCEPT

# Allow TAK federation
iptables -A INPUT -p tcp --dport 9000 -j ACCEPT
```

### Windows Firewall Rules
```powershell
# Enable TAK Server ports
New-NetFirewallRule -DisplayName "TAK TLS" -Direction Inbound -Protocol TCP -LocalPort 8089
New-NetFirewallRule -DisplayName "TAK TCP" -Direction Inbound -Protocol TCP -LocalPort 8087
New-NetFirewallRule -DisplayName "TAK HTTPS" -Direction Inbound -Protocol TCP -LocalPort 8443
New-NetFirewallRule -DisplayName "TAK WebTAK" -Direction Inbound -Protocol TCP -LocalPort 8446

# Enable TAK multicast
New-NetFirewallRule -DisplayName "TAK Mesh" -Direction Inbound -Protocol UDP -LocalPort 6969
```

## Quality of Service (QoS) Configuration

### Traffic Prioritization
| Traffic Type | DSCP | Priority | Bandwidth |
|--------------|------|----------|-----------|
| **CoT Messages** | AF41 (34) | High | 10-50 Kbps per user |
| **Video Streams** | AF31 (26) | Medium-High | 1-10 Mbps per stream |
| **Chat/Text** | AF21 (18) | Medium | 1-5 Kbps per user |
| **File Transfers** | AF11 (10) | Low | Bulk/Background |
| **Web Interface** | DF (0) | Best Effort | Variable |

### Network Requirements
- **Latency**: <100ms for real-time operations
- **Jitter**: <50ms for streaming data
- **Packet Loss**: <0.1% for reliable operations
- **Bandwidth**: 1 Mbps minimum per 50 concurrent users

## Security Configuration Standards

### TLS Configuration
```yaml
# Minimum TLS settings
tls_version: "1.2"  # TLS 1.2 minimum, 1.3 preferred
cipher_suites:
  - "ECDHE-ECDSA-AES256-GCM-SHA384"
  - "ECDHE-RSA-AES256-GCM-SHA384"
  - "ECDHE-ECDSA-AES128-GCM-SHA256"
certificate_verification: "strict"
hostname_verification: true
```

### Access Control Lists
```yaml
# TAK Server group configuration
groups:
  - name: "Command"
    permissions: ["read", "write", "admin"]
    filter_expression: "type LIKE 'a-f-*'"
  
  - name: "Operators"
    permissions: ["read", "write"]
    filter_expression: "NOT type LIKE 'b-a-*'"
  
  - name: "Observers"
    permissions: ["read"]
    filter_expression: "NOT classification = 'SECRET'"
```

## Troubleshooting Quick Guide

### Common Connection Issues
1. **Certificate Problems**
   - Verify certificate validity and chain
   - Check system time synchronization
   - Validate certificate format (PEM vs PKCS#12)

2. **Network Connectivity**
   - Test port connectivity: `telnet takserver.example.mil 8089`
   - Check firewall rules and NAT configuration
   - Verify DNS resolution

3. **Protocol Compatibility**
   - Confirm TAK server and client versions
   - Check protocol version support
   - Validate message format compliance

### Diagnostic Commands
```bash
# Test TAK server connectivity
nc -zv takserver.example.mil 8089

# Monitor TAK traffic
tcpdump -i any port 8089 or port 6969

# Check certificate details
openssl x509 -in client.pem -text -noout

# Validate TAK server response
curl -k https://takserver.example.mil:8443/Marti/api/version

# Test multicast reception
socat UDP4-RECVFROM:6969,ip-add-membership=239.2.3.1:eth0 -
```

## Performance Monitoring

### Key Metrics
- **Message Latency**: End-to-end CoT delivery time
- **Throughput**: Messages per second processed
- **Connection Count**: Active client connections
- **Memory Usage**: Server and client memory consumption
- **Network Utilization**: Bandwidth usage patterns

### Monitoring Tools
- **TAK Server Metrics**: Built-in JMX monitoring
- **Network Analysis**: Wireshark, tcpdump
- **Performance Testing**: Apache JMeter, custom tools
- **Log Analysis**: ELK Stack, Splunk
- **Health Checks**: Nagios, Zabbix

## Federation Configuration

### Cross-Domain Setup
```yaml
federation:
  outgoing_connections:
    - name: "coalition_server"
      address: "tak-coalition.example.mil"
      port: 9000
      protocol: "tls"
      certificate: "/opt/tak/certs/federation.p12"
      
  incoming_connections:
    enabled: true
    port: 9000
    certificate: "/opt/tak/certs/server.p12"
    
  filtering:
    outbound_policy: "whitelist"
    outbound_filter: "type LIKE 'a-f-*' AND NOT classification = 'SECRET'"
```

For complete technical specifications and implementation details, refer to the [comprehensive TAK compatibility documentation](./api-analysis/open-architecture-compatibility-analysis.md#tak-team-awareness-kit-compatibility-matrix).